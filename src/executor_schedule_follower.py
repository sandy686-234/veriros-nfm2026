#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
executor_schedule_follower.py

Schedule-following executor (导师要求版本).
This is a cleaned-up version of the original file with proper indentation
and a runnable `main()` entrypoint.
"""

import math, time, csv, json
from pathlib import Path

try:
    import HAL, Frequency
except ImportError:
    import hal_stub as HAL
    import frequency_stub as Frequency

# ================== AGENT CONFIG ==================
AGENT_NAME = "A"  # ← 修改为 "A" 或 "B"

# ========== CORE CONFIG ==========
HZ = 20
POS_TOL = 0.15
V_MAX, W_MAX = 0.7, 0.9

# --- STL/Safety config ---
DANGER_A = (3.0, 5.0, -0.5, 0.5)
VMAX_TABLE = [(10.0, 0.4), (20.0, 0.2), (1e9, 0.3)]
GOAL_WINDOW = 5.0

# --- Schedule following config ---
SCHEDULE_TOLERANCE = 2.0  # 允许的调度偏差(秒)
EARLY_START_MARGIN = 0.5  # 提前开始的容忍度(秒)

RUNS_DIR = "runs"
POSE_LOG_PERIOD = 1.0


def latest_schedule_path(runs_dir="runs"):
    cand = sorted(Path(runs_dir).glob("*/schedule.json"))
    return str(cand[-1]) if cand else None


def load_schedule(path=None):
    if path is None:
        path = latest_schedule_path()
    if not path or not Path(path).exists():
        print(f"[ERROR] No schedule found. Run gate_smt_scheduler.py first!")
        return None
    data = json.loads(Path(path).read_text())
    print(f"[SCHEDULE] Loaded from {path}")
    print(f"[SCHEDULE] Makespan: {data.get('makespan', 'N/A')}s")
    return data


# Load schedule at startup
SCHEDULE = load_schedule()
if SCHEDULE is None:
    print("[FATAL] Cannot proceed without schedule")
    exit(1)

MY_SCHEDULE = SCHEDULE.get("schedules", {}).get(AGENT_NAME, [])
if not MY_SCHEDULE:
    print(f"[ERROR] No schedule found for agent {AGENT_NAME}")
    exit(1)

print(f"[AGENT] {AGENT_NAME} has {len(MY_SCHEDULE)} tasks:")
for task in MY_SCHEDULE:
    print(f"  - {task['task_id']}: start={task['start_time']:.2f}s, "
          f"duration={task['duration']:.2f}s")

# Extract resource info
RESOURCES = SCHEDULE.get("resources", {})
CORRIDOR = None
for res_name, res_data in RESOURCES.items():
    if res_data.get("type") == "mutex":
        rect = res_data.get("rect")
        if rect:
            CORRIDOR = tuple(rect)
            break

if CORRIDOR:
    print(f"[RESOURCES] Corridor: {CORRIDOR}")


# Utilities
if hasattr(Frequency, "setHz"):
    Frequency.setHz(HZ)


def now():
    return time.monotonic()


def pose_xyth():
    if hasattr(HAL, "getPose2d"):
        p = HAL.getPose2d()
        try:
            x, y, th = p
        except Exception:
            x = getattr(p, "x", 0.0)
            y = getattr(p, "y", 0.0)
            th = getattr(p, "theta", getattr(p, "yaw", 0.0))
        return float(x), float(y), float(th)
    if hasattr(HAL, "getPose3d"):
        p = HAL.getPose3d()
        try:
            x, y, z, roll, pitch, yaw = p
            return float(x), float(y), float(yaw)
        except Exception:
            x = getattr(p, "x", 0.0)
            y = getattr(p, "y", 0.0)
            th = getattr(p, "yaw", getattr(p, "theta", 0.0))
            return float(x), float(y), float(th)
    return 0.0, 0.0, 0.0


def dist(a, b, c, d):
    return math.hypot(a - c, b - d)


def normalize_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def point_heading_ctrl(px, py, th, gx, gy):
    dx, dy = gx - px, gy - py
    target = math.atan2(dy, dx)
    e = normalize_angle(target - th)
    v = V_MAX * (1.0 - min(abs(e) / math.pi, 1.0))
    w = 2.0 * e
    v = max(-V_MAX, min(V_MAX, v))
    w = max(-W_MAX, min(W_MAX, w))
    return v, w


class OnlineMonitor:
    def __init__(self, horizon_sec, dt):
        import collections, math as _m
        self.maxlen = max(1, int(_m.ceil(horizon_sec / max(dt, 1e-6))))
        self.buf = collections.deque(maxlen=self.maxlen)

    def clear(self):
        self.buf.clear()

    def step(self, value):
        self.buf.append(float(value))

    def rob_G(self):
        return min(self.buf) if self.buf else float('inf')

    def rob_F(self):
        return max(self.buf) if self.buf else float('-inf')


def vmax_of(t_since):
    for t_up, v in VMAX_TABLE:
        if t_since < t_up:
            return v
    return VMAX_TABLE[-1][1] if VMAX_TABLE else V_MAX


def in_rect(px, py, rect):
    xmin, xmax, ymin, ymax = rect
    return (xmin <= px <= xmax) and (ymin <= py <= ymax)


def margin_not_in_A(px, py):
    return 0.05 if not in_rect(px, py, DANGER_A) else -0.05


def margin_in_goal(px, py, gx, gy, radius):
    return radius - dist(px, py, gx, gy)


def predict_enter_A(px, py, v, w, dt):
    th = pose_xyth()[2]
    nx = px + v * math.cos(th) * dt
    ny = py + v * math.sin(th) * dt
    return in_rect(nx, ny, DANGER_A)


def export_header(w):
    w.writerow([
        "t", "event", "task", "x", "y", "scheduled_start", "actual_start",
        "schedule_deviation", "note",
        "phi1_bool", "phi1_rob",
        "phi2_bool", "phi2_rob",
        "phi3_bool", "phi3_rob",
        "phi_all_bool", "phi_all_rob",
        "speed", "vmax"
    ])


def export_event(w, t, ev, task_id="", x=None, y=None,
                 scheduled_start=None, actual_start=None, note="",
                 phi1_rob=None, phi2_rob=None, phi3_rob=None,
                 speed=None, vmax=None):
    def b(v):
        return "" if v is None else (1 if v >= 0 else 0)

    phi_all_rob = None
    phi_all_bool = ""
    if phi1_rob is not None and phi2_rob is not None and phi3_rob is not None:
        phi_all_rob = min(phi1_rob, phi2_rob, phi3_rob)
        phi_all_bool = 1 if (phi1_rob >= 0 and phi2_rob >= 0 and phi3_rob >= 0) else 0

    sched_dev = ""
    if scheduled_start is not None and actual_start is not None:
        sched_dev = f"{abs(actual_start - scheduled_start):.3f}"

    w.writerow([
        f"{t:.3f}",
        ev,
        task_id,
        (f"{x:.3f}" if x is not None else ""),
        (f"{y:.3f}" if y is not None else ""),
        (f"{scheduled_start:.3f}" if scheduled_start is not None else ""),
        (f"{actual_start:.3f}" if actual_start is not None else ""),
        sched_dev,
        note,
        (b(phi1_rob) if phi1_rob is not None else ""),
        (f"{phi1_rob:.4f}" if phi1_rob is not None else ""),
        (b(phi2_rob) if phi2_rob is not None else ""),
        (f"{phi2_rob:.4f}" if phi2_rob is not None else ""),
        (b(phi3_rob) if phi3_rob is not None else ""),
        (f"{phi3_rob:.4f}" if phi3_rob is not None else ""),
        (phi_all_bool if phi_all_rob is not None else ""),
        (f"{phi_all_rob:.4f}" if phi_all_rob is not None else ""),
        (f"{speed:.3f}" if speed is not None else ""),
        (f"{vmax:.3f}" if vmax is not None else ""),
    ])


def main():
    # Runtime start time (wall clock)
    t0_rt = now()
    # Setup trace logging
    run_ts = time.strftime("%Y%m%d-%H%M%S")
    run_dir = Path(RUNS_DIR) / run_ts
    run_dir.mkdir(parents=True, exist_ok=True)
    trace_path = run_dir / f"trace_{AGENT_NAME}.csv"

    trace_fp = open(trace_path, "w", newline="")
    trace = csv.writer(trace_fp)
    export_header(trace)

    # STL monitors
    DT = 1.0 / HZ
    mon_not_inA = OnlineMonitor(horizon_sec=2.0, dt=DT)
    mon_inGoal = OnlineMonitor(horizon_sec=GOAL_WINDOW, dt=DT)
    mon_vlimit = OnlineMonitor(horizon_sec=0.25, dt=DT)

    # Schedule following state
    current_task_idx = 0
    current_task = None
    task_actual_start = None
    last_pose_log = t0_rt
    _prev_pose = None

    print(f"\n[START] Agent {AGENT_NAME} following schedule...")
    print(f"[START] t0_rt = {t0_rt:.3f}")

    export_event(trace, 0.0, "start", note="Schedule-following execution with STL")

    try:
        while True:
            t = now()
            t_rel = t - t0_rt  # Relative time since runtime start
            x, y, th = pose_xyth()

            # Speed estimation
            if _prev_pose is None:
                spd = 0.0
            else:
                px, py, pt = _prev_pose
                spd = dist(x, y, px, py) / max(1e-6, t - pt)
            _prev_pose = (x, y, t)

            # Speed limit and STL margins
            vmax_now = vmax_of(t_rel)
            mon_not_inA.step(margin_not_in_A(x, y))

            if current_task:
                gx, gy = current_task["goal"]
                mon_inGoal.step(margin_in_goal(x, y, gx, gy, POS_TOL))
            else:
                mon_inGoal.step(-1e6)

            mon_vlimit.step(vmax_now - spd)

            # Check if all tasks done
            if current_task_idx >= len(MY_SCHEDULE):
                HAL.setV(0.0)
                HAL.setW(0.0)
                if (t - last_pose_log) >= POSE_LOG_PERIOD:
                    export_event(trace, t_rel, "idle", note="All tasks complete",
                                 x=x, y=y,
                                 phi1_rob=mon_not_inA.rob_G(),
                                 phi2_rob=mon_inGoal.rob_F(),
                                 phi3_rob=mon_vlimit.rob_G(),
                                 speed=spd, vmax=vmax_now)
                    last_pose_log = t

                if hasattr(Frequency, "tick"):
                    Frequency.tick()
                continue

            # Get current scheduled task
            scheduled_task = MY_SCHEDULE[current_task_idx]
            scheduled_start = scheduled_task["start_time"]
            scheduled_end = scheduled_start + scheduled_task["duration"]
            task_id = scheduled_task["task_id"]
            gx, gy = scheduled_task["goal"]

            # WAIT UNTIL SCHEDULED START TIME
            if t_rel < (scheduled_start - EARLY_START_MARGIN):
                HAL.setV(0.0)
                HAL.setW(0.0)
                wait_time = scheduled_start - t_rel
                if (t - last_pose_log) >= POSE_LOG_PERIOD:
                    export_event(trace, t_rel, "waiting", task_id=task_id,
                                 x=x, y=y,
                                 scheduled_start=scheduled_start,
                                 note=f"Waiting {wait_time:.1f}s until scheduled start",
                                 phi1_rob=mon_not_inA.rob_G(),
                                 phi2_rob=mon_inGoal.rob_F(),
                                 phi3_rob=mon_vlimit.rob_G(),
                                 speed=spd, vmax=vmax_now)
                    last_pose_log = t

                if hasattr(Frequency, "tick"):
                    Frequency.tick()
                continue

            # START TASK (if not started yet)
            if current_task != scheduled_task:
                current_task = scheduled_task
                task_actual_start = t_rel
                mon_inGoal.clear()
                deviation = abs(task_actual_start - scheduled_start)
                export_event(trace, t_rel, "task_start", task_id=task_id,
                             x=x, y=y,
                             scheduled_start=scheduled_start,
                             actual_start=task_actual_start,
                             note=f"Deviation: {deviation:.3f}s",
                             phi1_rob=mon_not_inA.rob_G(),
                             phi2_rob=mon_inGoal.rob_F(),
                             phi3_rob=mon_vlimit.rob_G(),
                             speed=spd, vmax=vmax_now)
                if deviation > SCHEDULE_TOLERANCE:
                    print(f"[WARNING] Task {task_id} started {deviation:.2f}s off schedule!")

            # CHECK FOR SCHEDULE OVERRUN
            if t_rel > scheduled_end:
                overrun = t_rel - scheduled_end
                if (t - last_pose_log) >= POSE_LOG_PERIOD:
                    export_event(trace, t_rel, "overrun", task_id=task_id,
                                 x=x, y=y,
                                 scheduled_start=scheduled_start,
                                 actual_start=task_actual_start,
                                 note=f"OVERRUN by {overrun:.2f}s",
                                 phi1_rob=mon_not_inA.rob_G(),
                                 phi2_rob=mon_inGoal.rob_F(),
                                 phi3_rob=mon_vlimit.rob_G(),
                                 speed=spd, vmax=vmax_now)
                    last_pose_log = t

            # EXECUTE TASK WITH SAFETY FENCE
            v_cmd, w_cmd = point_heading_ctrl(x, y, th, gx, gy)
            v_cmd = max(-vmax_now, min(vmax_now, v_cmd))
            fence_note = ""
            if predict_enter_A(x, y, v_cmd, w_cmd, 1.0 / HZ):
                v_out, w_out = 0.0, 0.0
                fence_note = "FENCE: blocked entry to danger zone"
            else:
                v_out, w_out = v_cmd, w_cmd
            HAL.setV(v_out)
            HAL.setW(w_out)

            # CHECK TASK COMPLETION
            if dist(x, y, gx, gy) <= POS_TOL:
                task_completion_time = t_rel
                task_duration = task_completion_time - task_actual_start
                export_event(trace, t_rel, "task_complete", task_id=task_id,
                             x=x, y=y,
                             scheduled_start=scheduled_start,
                             actual_start=task_actual_start,
                             note=f"Completed in {task_duration:.2f}s "
                                  f"(scheduled: {scheduled_task['duration']:.2f}s)",
                             phi1_rob=mon_not_inA.rob_G(),
                             phi2_rob=mon_inGoal.rob_F(),
                             phi3_rob=mon_vlimit.rob_G(),
                             speed=spd, vmax=vmax_now)
                current_task_idx += 1
                current_task = None
                task_actual_start = None
                HAL.setV(0.0)
                HAL.setW(0.0)

            # PERIODIC LOGGING
            if (t - last_pose_log) >= POSE_LOG_PERIOD:
                export_event(trace, t_rel, "pose", task_id=task_id,
                             x=x, y=y,
                             scheduled_start=scheduled_start,
                             actual_start=task_actual_start,
                             note=fence_note,
                             phi1_rob=mon_not_inA.rob_G(),
                             phi2_rob=mon_inGoal.rob_F(),
                             phi3_rob=mon_vlimit.rob_G(),
                             speed=spd, vmax=vmax_now)
                last_pose_log = t

            if hasattr(Frequency, "tick"):
                Frequency.tick()

    finally:
        HAL.setV(0.0)
        HAL.setW(0.0)
        export_event(trace, now() - t0_rt, "end", note="Execution complete")
        trace_fp.flush()
        trace_fp.close()
        print(f"\n[DONE] Trace saved to {trace_path}")


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
executor_schedule_follower.py

Schedule-following executor (导师要求版本):
- 加载SMT生成的schedule.json
- 严格按照预定的开始时间执行任务
- 不做在线调度决策（no EDF）
- 保留STL监控和safety fence
"""

import math, time, csv, json
from pathlib import Path

try:
    import HAL, Frequency
except ImportError:
    import hal_stub as HAL
    import frequency_stub as Frequency

# ================== AGENT CONFIG ==================
AGENT_NAME = "A"  # ← 修改为 "A" 或 "B"

# ========== CORE CONFIG ==========
HZ = 20
POS_TOL = 0.15
V_MAX, W_MAX = 0.7, 0.9

# --- STL/Safety config ---
DANGER_A = (3.0, 5.0, -0.5, 0.5)
VMAX_TABLE = [(10.0, 0.4), (20.0, 0.2), (1e9, 0.3)]
GOAL_WINDOW = 5.0

# --- Schedule following config ---
SCHEDULE_TOLERANCE = 2.0  # 允许的调度偏差(秒)
EARLY_START_MARGIN = 0.5  # 提前开始的容忍度(秒)

RUNS_DIR = "runs"
POSE_LOG_PERIOD = 1.0

# ==========================================================
# SCHEDULE LOADING
# ==========================================================

def latest_schedule_path(runs_dir="runs"):
    """Find most recent schedule.json"""
    cand = sorted(Path(runs_dir).glob("*/schedule.json"))
    return str(cand[-1]) if cand else None


def load_schedule(path=None):
    """
    Load SMT-generated schedule
    
    Expected format:
    {
        "t0_mono": <gate time>,
        "makespan": <total time>,
        "schedules": {
            "A": [
                {"task_id": "A_leg0", "start_time": 0.0, "duration": 8.5, ...},
                ...
            ],
            "B": [...]
        },
        "resources": {...}
    }
    """
    if path is None:
        path = latest_schedule_path()
    
    if not path or not Path(path).exists():
        print(f"[ERROR] No schedule found. Run gate_smt_scheduler.py first!")
        return None
    
    data = json.loads(Path(path).read_text())
    print(f"[SCHEDULE] Loaded from {path}")
    print(f"[SCHEDULE] Makespan: {data.get('makespan', 'N/A')}s")
    
    return data


# Load schedule at startup
SCHEDULE = load_schedule()

if SCHEDULE is None:
    print("[FATAL] Cannot proceed without schedule")
    exit(1)

# Extract this agent's schedule
MY_SCHEDULE = SCHEDULE["schedules"].get(AGENT_NAME, [])

if not MY_SCHEDULE:
    print(f"[ERROR] No schedule found for agent {AGENT_NAME}")
    exit(1)

print(f"[AGENT] {AGENT_NAME} has {len(MY_SCHEDULE)} tasks:")
for task in MY_SCHEDULE:
    print(f"  - {task['task_id']}: start={task['start_time']:.2f}s, "
          f"duration={task['duration']:.2f}s")

# Extract resource info
RESOURCES = SCHEDULE.get("resources", {})
CORRIDOR = None
for res_name, res_data in RESOURCES.items():
    if res_data["type"] == "mutex":
        CORRIDOR = tuple(res_data["rect"])
        break

if CORRIDOR:
    print(f"[RESOURCES] Corridor: {CORRIDOR}")

# ==========================================================
# UTILITIES
# ==========================================================

if hasattr(Frequency, "setHz"):
    Frequency.setHz(HZ)


def now():
    return time.monotonic()


def pose_xyth():
    if hasattr(HAL, "getPose2d"):
        p = HAL.getPose2d()
        try:
            x, y, th = p
        except Exception:
            x = getattr(p, "x", 0.0)
            y = getattr(p, "y", 0.0)
            th = getattr(p, "theta", getattr(p, "yaw", 0.0))
        return float(x), float(y), float(th)
    if hasattr(HAL, "getPose3d"):
        p = HAL.getPose3d()
        try:
            x, y, z, roll, pitch, yaw = p
            return float(x), float(y), float(yaw)
        except Exception:
            x = getattr(p, "x", 0.0)
            y = getattr(p, "y", 0.0)
            th = getattr(p, "yaw", getattr(p, "theta", 0.0))
            return float(x), float(y), float(th)
    return 0.0, 0.0, 0.0


def dist(a, b, c, d):
    return math.hypot(a - c, b - d)


def normalize_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def point_heading_ctrl(px, py, th, gx, gy):
    dx, dy = gx - px, gy - py
    target = math.atan2(dy, dx)
    e = normalize_angle(target - th)
    v = V_MAX * (1.0 - min(abs(e) / math.pi, 1.0))
    w = 2.0 * e
    v = max(-V_MAX, min(V_MAX, v))
    w = max(-W_MAX, min(W_MAX, w))
    return v, w


# --------- STL Monitors ---------
class OnlineMonitor:
    def __init__(self, horizon_sec, dt):
        import collections, math as _m
        self.maxlen = max(1, int(_m.ceil(horizon_sec / max(dt, 1e-6))))
        self.buf = collections.deque(maxlen=self.maxlen)

    def clear(self):
        self.buf.clear()

    def step(self, value):
        self.buf.append(float(value))

    def rob_G(self):
        return min(self.buf) if self.buf else float('inf')

    def rob_F(self):
        return max(self.buf) if self.buf else float('-inf')


def vmax_of(t_since):
    for t_up, v in VMAX_TABLE:
        if t_since < t_up:
            return v
    return VMAX_TABLE[-1][1] if VMAX_TABLE else V_MAX


def in_rect(px, py, rect):
    xmin, xmax, ymin, ymax = rect
    return (xmin <= px <= xmax) and (ymin <= py <= ymax)


def margin_not_in_A(px, py):
    return 0.05 if not in_rect(px, py, DANGER_A) else -0.05


def margin_in_goal(px, py, gx, gy, radius):
    return radius - dist(px, py, gx, gy)


def predict_enter_A(px, py, v, w, dt):
    th = pose_xyth()[2]
    nx = px + v * math.cos(th) * dt
    ny = py + v * math.sin(th) * dt
    return in_rect(nx, ny, DANGER_A)


# --------- Trace Export ---------
def export_header(w):
    w.writerow([
        "t", "event", "task", "x", "y", "scheduled_start", "actual_start",
        "schedule_deviation", "note",
        "phi1_bool", "phi1_rob",
        "phi2_bool", "phi2_rob",
        "phi3_bool", "phi3_rob",
        "phi_all_bool", "phi_all_rob",
        "speed", "vmax"
    ])


def export_event(w, t, ev, task_id="", x=None, y=None,
                 scheduled_start=None, actual_start=None, note="",
                 phi1_rob=None, phi2_rob=None, phi3_rob=None,
                 speed=None, vmax=None):
    def b(v):
        return "" if v is None else (1 if v >= 0 else 0)

    phi_all_rob = None
    phi_all_bool = ""
    if phi1_rob is not None and phi2_rob is not None and phi3_rob is not None:
        phi_all_rob = min(phi1_rob, phi2_rob, phi3_rob)
        phi_all_bool = 1 if (phi1_rob >= 0 and phi2_rob >= 0 and phi3_rob >= 0) else 0

    sched_dev = ""
    if scheduled_start is not None and actual_start is not None:
        sched_dev = f"{abs(actual_start - scheduled_start):.3f}"

    w.writerow([
        f"{t:.3f}",
        ev,
        task_id,
        (f"{x:.3f}" if x is not None else ""),
        (f"{y:.3f}" if y is not None else ""),
        (f"{scheduled_start:.3f}" if scheduled_start is not None else ""),
        (f"{actual_start:.3f}" if actual_start is not None else ""),
        sched_dev,
        note,
        (b(phi1_rob) if phi1_rob is not None else ""),
        (f"{phi1_rob:.4f}" if phi1_rob is not None else ""),
        (b(phi2_rob) if phi2_rob is not None else ""),
        (f"{phi2_rob:.4f}" if phi2_rob is not None else ""),
        (b(phi3_rob) if phi3_rob is not None else ""),
        (f"{phi3_rob:.4f}" if phi3_rob is not None else ""),
        (phi_all_bool if phi_all_rob is not None else ""),
        (f"{phi_all_rob:.4f}" if phi_all_rob is not None else ""),
        (f"{speed:.3f}" if speed is not None else ""),
        (f"{vmax:.3f}" if vmax is not None else ""),
    ])
def main():
    # Runtime start time (wall clock)
    t0_rt = now()
    # Setup trace logging
    run_ts = time.strftime("%Y%m%d-%H%M%S")
    run_dir = Path(RUNS_DIR) / run_ts
    run_dir.mkdir(parents=True, exist_ok=True)
    trace_path = run_dir / f"trace_{AGENT_NAME}.csv"

    trace_fp = open(trace_path, "w", newline="")
    trace = csv.writer(trace_fp)
    export_header(trace)

    # STL monitors
    DT = 1.0 / HZ
    mon_not_inA = OnlineMonitor(horizon_sec=2.0, dt=DT)
    mon_inGoal = OnlineMonitor(horizon_sec=GOAL_WINDOW, dt=DT)
    mon_vlimit = OnlineMonitor(horizon_sec=0.25, dt=DT)

    # Schedule following state
    current_task_idx = 0
    current_task = None
    task_actual_start = None
    last_pose_log = t0_rt
    _prev_pose = None

    print(f"\n[START] Agent {AGENT_NAME} following schedule...")
    print(f"[START] t0_rt = {t0_rt:.3f}")

    export_event(trace, 0.0, "start", note="Schedule-following execution with STL")

    try:
        while True:
            t = now()
            t_rel = t - t0_rt  # Relative time since runtime start
            x, y, th = pose_xyth()

            # Speed estimation
            if _prev_pose is None:
                spd = 0.0
            else:
                px, py, pt = _prev_pose
                spd = dist(x, y, px, py) / max(1e-6, t - pt)
            _prev_pose = (x, y, t)

            # Speed limit and STL margins
            vmax_now = vmax_of(t_rel)
            mon_not_inA.step(margin_not_in_A(x, y))

            if current_task:
                gx, gy = current_task["goal"]
                mon_inGoal.step(margin_in_goal(x, y, gx, gy, POS_TOL))
            else:
                mon_inGoal.step(-1e6)

            mon_vlimit.step(vmax_now - spd)

            # ============================================
            # SCHEDULE FOLLOWING LOGIC (核心修改)
            # ============================================

            # Check if all tasks done
            if current_task_idx >= len(MY_SCHEDULE):
                HAL.setV(0.0)
                HAL.setW(0.0)
                if (t - last_pose_log) >= POSE_LOG_PERIOD:
                    export_event(trace, t_rel, "idle", note="All tasks complete",
                                 x=x, y=y,
                                 phi1_rob=mon_not_inA.rob_G(),
                                 phi2_rob=mon_inGoal.rob_F(),
                                 phi3_rob=mon_vlimit.rob_G(),
                                 speed=spd, vmax=vmax_now)
                    last_pose_log = t

                if hasattr(Frequency, "tick"):
                    Frequency.tick()
                continue

            # Get current scheduled task
            scheduled_task = MY_SCHEDULE[current_task_idx]
            scheduled_start = scheduled_task["start_time"]
            scheduled_end = scheduled_start + scheduled_task["duration"]
            task_id = scheduled_task["task_id"]
            gx, gy = scheduled_task["goal"]

            # ============================================
            # WAIT UNTIL SCHEDULED START TIME
            # ============================================
            if t_rel < (scheduled_start - EARLY_START_MARGIN):
                # Too early - wait
                HAL.setV(0.0)
                HAL.setW(0.0)

                wait_time = scheduled_start - t_rel
                if (t - last_pose_log) >= POSE_LOG_PERIOD:
                    export_event(trace, t_rel, "waiting", task_id=task_id,
                                 x=x, y=y,
                                 scheduled_start=scheduled_start,
                                 note=f"Waiting {wait_time:.1f}s until scheduled start",
                                 phi1_rob=mon_not_inA.rob_G(),
                                 phi2_rob=mon_inGoal.rob_F(),
                                 phi3_rob=mon_vlimit.rob_G(),
                                 speed=spd, vmax=vmax_now)
                    last_pose_log = t

                if hasattr(Frequency, "tick"):
                    Frequency.tick()
                continue

            # ============================================
            # START TASK (if not started yet)
            # ============================================
            if current_task != scheduled_task:
                current_task = scheduled_task
                task_actual_start = t_rel
                mon_inGoal.clear()

                deviation = abs(task_actual_start - scheduled_start)

                export_event(trace, t_rel, "task_start", task_id=task_id,
                             x=x, y=y,
                             scheduled_start=scheduled_start,
                             actual_start=task_actual_start,
                             note=f"Deviation: {deviation:.3f}s",
                             phi1_rob=mon_not_inA.rob_G(),
                             phi2_rob=mon_inGoal.rob_F(),
                             phi3_rob=mon_vlimit.rob_G(),
                             speed=spd, vmax=vmax_now)

                if deviation > SCHEDULE_TOLERANCE:
                    print(f"[WARNING] Task {task_id} started {deviation:.2f}s off schedule!")

            # ============================================
            # CHECK FOR SCHEDULE OVERRUN
            # ============================================
        if t_rel > scheduled_end:
            overrun = t_rel - scheduled_end
            if (t - last_pose_log) >= POSE_LOG_PERIOD:
                export_event(trace, t_rel, "overrun", task_id=task_id,
                             x=x, y=y,
                             scheduled_start=scheduled_start,
                             actual_start=task_actual_start,
                             note=f"OVERRUN by {overrun:.2f}s",
                             phi1_rob=mon_not_inA.rob_G(),
                             phi2_rob=mon_inGoal.rob_F(),
                             phi3_rob=mon_vlimit.rob_G(),
                             speed=spd, vmax=vmax_now)
                last_pose_log = t
        
        # ============================================
        # EXECUTE TASK WITH SAFETY FENCE
        # ============================================
        v_cmd, w_cmd = point_heading_ctrl(x, y, th, gx, gy)
        v_cmd = max(-vmax_now, min(vmax_now, v_cmd))
        
        # HCD Safety Fence
        fence_note = ""
        if predict_enter_A(x, y, v_cmd, w_cmd, 1.0 / HZ):
            v_out, w_out = 0.0, 0.0
            fence_note = "FENCE: blocked entry to danger zone"
        else:
            v_out, w_out = v_cmd, w_cmd
        
        HAL.setV(v_out)
        HAL.setW(w_out)
        
        # ============================================
        # CHECK TASK COMPLETION
        # ============================================
        if dist(x, y, gx, gy) <= POS_TOL:
            task_completion_time = t_rel
            task_duration = task_completion_time - task_actual_start
            
            export_event(trace, t_rel, "task_complete", task_id=task_id,
                         x=x, y=y,
                         scheduled_start=scheduled_start,
                         actual_start=task_actual_start,
                         note=f"Completed in {task_duration:.2f}s "
                              f"(scheduled: {scheduled_task['duration']:.2f}s)",
                         phi1_rob=mon_not_inA.rob_G(),
                         phi2_rob=mon_inGoal.rob_F(),
                         phi3_rob=mon_vlimit.rob_G(),
                         speed=spd, vmax=vmax_now)
            
            # Move to next task
            current_task_idx += 1
            current_task = None
            task_actual_start = None
            
            HAL.setV(0.0)
            HAL.setW(0.0)
        
        # ============================================
        # PERIODIC LOGGING
        # ============================================
        if (t - last_pose_log) >= POSE_LOG_PERIOD:
            export_event(trace, t_rel, "pose", task_id=task_id,
                         x=x, y=y,
                         scheduled_start=scheduled_start,
                         actual_start=task_actual_start,
                         note=fence_note,
                         phi1_rob=mon_not_inA.rob_G(),
                         phi2_rob=mon_inGoal.rob_F(),
                         phi3_rob=mon_vlimit.rob_G(),
                         speed=spd, vmax=vmax_now)
            last_pose_log = t
        
        if hasattr(Frequency, "tick"):
            Frequency.tick()

    finally:
        HAL.setV(0.0)
        HAL.setW(0.0)
        export_event(trace, now() - t0_rt, "end", note="Execution complete")
        trace_fp.flush()
        trace_fp.close()
        print(f"\n[DONE] Trace saved to {trace_path}")



