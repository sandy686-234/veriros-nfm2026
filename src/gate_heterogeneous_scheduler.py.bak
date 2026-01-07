#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import json
import subprocess
import argparse
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    print("[ERROR] yaml module not found. Install with: pip install pyyaml")
    sys.exit(1)


def load_config(config_file):
    config_path = Path(config_file)
    
    if not config_path.exists():
        config_path = Path(".") / config_file
        if not config_path.exists():
            print(f"[ERROR] Config file not found: {config_file}")
            print(f"Working directory: {Path('.').absolute()}")
            sys.exit(1)
    
    try:
        with open(config_path) as f:
            content = f.read()
        
        if not content.strip():
            print(f"[ERROR] Config file is empty: {config_path}")
            sys.exit(1)
        
        config = yaml.safe_load(content)
        
        if config is None:
            print(f"[ERROR] Invalid YAML file: {config_path}")
            sys.exit(1)
        
        if not isinstance(config, dict):
            print(f"[ERROR] Config must be a dictionary")
            sys.exit(1)
        
        print(f"[CONFIG] Loaded from {config_path.absolute()}")
        return config
    
    except yaml.YAMLError as e:
        print(f"[ERROR] YAML parse error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"[ERROR] Failed to read config: {e}")
        sys.exit(1)


def build_task_pool(config):
    task_pool = {}
    tasks = config.get('tasks', [])
    
    if not tasks:
        print("[WARNING] No tasks in config")
        return task_pool
    
    for task in tasks:
        task_id = task.get('id')
        if not task_id:
            print("[WARNING] Task missing 'id' field")
            continue
        
        task_pool[task_id] = {
            "location": tuple(task.get('location', (0.0, 0.0))),
            "duration": float(task.get('duration', 0.0)),
            "deadline": float(task.get('deadline', float('inf'))),
            "requires_capability": task.get('requires_capability', None),
            "uses_resources": task.get('uses_resources', []),
            "description": task.get('description', task_id),
        }
    
    print(f"[CONFIG] Loaded {len(task_pool)} tasks")
    return task_pool


def build_robots(config):
    robots = {}
    robot_list = config.get('robots', [])
    
    if not robot_list:
        print("[WARNING] No robots in config")
        return robots
    
    for robot in robot_list:
        name = robot.get('name')
        if not name:
            print("[WARNING] Robot missing 'name' field")
            continue
        
        robots[name] = {
            "id": robot.get('id', name[0]),
            "capabilities": robot.get('capabilities', []),
            "max_speed": float(robot.get('max_speed', 0.5)),
            "start_position": tuple(robot.get('start_position', (0.0, 0.0))),
            "description": robot.get('description', name),
        }
    
    print(f"[CONFIG] Loaded {len(robots)} robots")
    return robots


def build_resources(config):
    resources = {}
    resource_dict = config.get('resources', {})
    
    if not resource_dict:
        print("[WARNING] No resources in config")
        return resources
    
    for res_name, res_data in resource_dict.items():
        resources[res_name] = {
            "type": res_data.get('type', 'mutex'),
            "location": tuple(res_data.get('location', (0.0, 0.0))),
            "traversal_time": float(res_data.get('traversal_time', 10.0)),
        }
    
    print(f"[CONFIG] Loaded {len(resources)} resources")
    return resources


EPSILON = 0.1
RUNS_DIR = "runs"
SOLVER_TIMEOUT = 120


def compute_travel_time(loc1, loc2, speed):
    distance = math.hypot(loc2[0] - loc1[0], loc2[1] - loc1[1])
    if speed <= 0:
        return 0.0
    return distance / speed


def validate_configuration(task_pool, robots):
    print("[CONFIG] Validating...")
    
    all_valid = True
    for task_id, task_data in task_pool.items():
        req_cap = task_data.get("requires_capability")
        if req_cap:
            capable_robots = [
                name for name, rdata in robots.items()
                if req_cap in rdata["capabilities"]
            ]
            if not capable_robots:
                print(f"[ERROR] Task '{task_id}' requires '{req_cap}', no robot available")
                all_valid = False
            else:
                print(f"  [{req_cap}] {task_id} -> {capable_robots}")
    
    if all_valid:
        print(f"[CONFIG] Valid: {len(task_pool)} tasks, {len(robots)} robots")
    return all_valid


def export_smt2_heterogeneous(task_pool, robots, resources, output_path):
    lines = []
    lines.append("(set-logic QF_LIRA)")
    lines.append("(set-option :produce-models true)")
    lines.append("(set-option :produce-unsat-cores true)")
    lines.append("(set-option :pp.decimal true)")
    lines.append("")
    
    task_ids = list(task_pool.keys())
    robot_ids = [r["id"] for r in robots.values()]
    
    lines.append("; Start times")
    for task_id in task_ids:
        lines.append(f"(declare-const s_{task_id} Real)")
        lines.append(f"(assert (>= s_{task_id} 0.0))")
    lines.append("")
    
    lines.append("; Assignment variables")
    for task_id in task_ids:
        for robot_id in robot_ids:
            var_name = f"assign_{task_id}_{robot_id}"
            lines.append(f"(declare-const {var_name} Int)")
            lines.append(f"(assert (or (= {var_name} 0) (= {var_name} 1)))")
    lines.append("")
    
    lines.append("; Ordering variables")
    for robot_id in robot_ids:
        for i, t1 in enumerate(task_ids):
            for t2 in task_ids[i+1:]:
                var_name = f"order_{t1}_{t2}_{robot_id}"
                lines.append(f"(declare-const {var_name} Int)")
                lines.append(f"(assert (or (= {var_name} 0) (= {var_name} 1)))")
    lines.append("")
    
    lines.append("(declare-const makespan Real)")
    lines.append("(assert (>= makespan 0.0))")
    lines.append("")
    
    lines.append("; C1: Each task assigned to exactly one robot")
    for task_id in task_ids:
        assign_vars = [f"assign_{task_id}_{rid}" for rid in robot_ids]
        sum_expr = " ".join(assign_vars)
        lines.append(f"(assert (= (+ {sum_expr}) 1))")
    lines.append("")
    
    lines.append("; C2: Capability constraints")
    for task_id, task_data in task_pool.items():
        req_cap = task_data.get("requires_capability")
        if req_cap:
            for robot_name, robot_data in robots.items():
                robot_id = robot_data["id"]
                if req_cap not in robot_data["capabilities"]:
                    lines.append(f"(assert (= assign_{task_id}_{robot_id} 0))")
    lines.append("")
    
    lines.append("; C3: Tasks on same robot non-overlapping")
    for robot_name, robot_data in robots.items():
        robot_id = robot_data["id"]
        robot_speed = robot_data["max_speed"]
        robot_start = robot_data["start_position"]
        
        for task_id, task_data in task_pool.items():
            initial_travel = compute_travel_time(robot_start, task_data["location"], robot_speed)
            lines.append(
                f"(assert (=> (= assign_{task_id}_{robot_id} 1) "
                f"(>= s_{task_id} {initial_travel:.2f})))"
            )
        
        for i, t1 in enumerate(task_ids):
            for t2 in task_ids[i+1:]:
                t1_data = task_pool[t1]
                t2_data = task_pool[t2]
                
                travel_12 = compute_travel_time(t1_data["location"], t2_data["location"], robot_speed)
                travel_21 = compute_travel_time(t2_data["location"], t1_data["location"], robot_speed)
                
                lines.append(
                    f"(assert (=> "
                    f"(and (= assign_{t1}_{robot_id} 1) "
                    f"(= assign_{t2}_{robot_id} 1) "
                    f"(= order_{t1}_{t2}_{robot_id} 1)) "
                    f"(>= s_{t2} (+ s_{t1} {t1_data['duration']:.2f} {travel_12:.2f}))))"
                )
                
                lines.append(
                    f"(assert (=> "
                    f"(and (= assign_{t1}_{robot_id} 1) "
                    f"(= assign_{t2}_{robot_id} 1) "
                    f"(= order_{t1}_{t2}_{robot_id} 0)) "
                    f"(>= s_{t1} (+ s_{t2} {t2_data['duration']:.2f} {travel_21:.2f}))))"
                )
    lines.append("")
    
    lines.append("; C4: Deadline constraints")
    for task_id, task_data in task_pool.items():
        deadline = task_data["deadline"]
        duration = task_data["duration"]
        lines.append(f"(assert (<= (+ s_{task_id} {duration:.2f}) {deadline:.2f}))")
    lines.append("")
    
    lines.append("; C5: Resource mutual exclusion")
    for res_name, res_data in resources.items():
        users = [tid for tid, tdata in task_pool.items() 
                 if res_name in tdata.get("uses_resources", [])]
        
        if len(users) < 2:
            continue
        
        for i, t1 in enumerate(users):
            for t2 in users[i+1:]:
                d1 = task_pool[t1]["duration"]
                d2 = task_pool[t2]["duration"]
                
                lines.append(
                    f"(assert (or "
                    f"(<= (+ s_{t1} {d1:.2f} {EPSILON:.2f}) s_{t2}) "
                    f"(<= (+ s_{t2} {d2:.2f} {EPSILON:.2f}) s_{t1})))"
                )
    lines.append("")
    
    lines.append("; C6: Makespan definition")
    for task_id, task_data in task_pool.items():
        duration = task_data["duration"]
        lines.append(f"(assert (>= makespan (+ s_{task_id} {duration:.2f})))")
    lines.append("")
    
    lines.append("(minimize makespan)")
    lines.append("")
    lines.append("(check-sat)")
    lines.append("(get-model)")
    
    Path(output_path).write_text("\n".join(lines))
    print(f"[SMT] Generated {len(lines)} lines -> {output_path}")
    return output_path


def run_z3_solver(smt2_path, timeout=120):
    try:
        result = subprocess.run(
            ["z3", f"-T:{timeout}", "-smt2", str(smt2_path)],
            capture_output=True,
            text=True,
            timeout=timeout + 10,
        )
        return True, result.stdout
    except FileNotFoundError:
        return None, "Z3 not found. Install: apt-get install z3 or brew install z3"
    except subprocess.TimeoutExpired:
        return False, f"Timeout after {timeout}s"
    except Exception as e:
        return False, f"Error: {str(e)}"


def parse_z3_result(z3_output, task_pool, robots):
    import re
    from fractions import Fraction

    lines = z3_output.strip().split("\n")

    if not lines or lines[0].strip() != "sat":
        return None

    values = {}

    def _parse_num(token: str):
        token = token.strip()
        m = re.match(r'\(\s*/\s*([^\s\)]+)\s+([^\s\)]+)\s*\)', token)
        if m:
            try:
                num = float(Fraction(m.group(1)))
                den = float(Fraction(m.group(2)))
                return num / den if den != 0 else None
            except Exception:
                return None
        if "/" in token:
            try:
                return float(Fraction(token))
            except Exception:
                pass
        try:
            return float(token)
        except Exception:
            return None

    for m in re.finditer(r'\(define-fun\s+([^\s\)]+)\s*\(\)\s*\w+\s*([^\)]+)\)', z3_output, re.DOTALL):
        name = m.group(1)
        val_str = m.group(2).strip()
        parsed = _parse_num(val_str)
        values[name] = parsed if parsed is not None else val_str

    after_sat = z3_output.split("sat", 1)[-1]
    pairs = re.findall(r'\(\s*([a-zA-Z0-9_]+)\s+([^\)\s]+)\s*\)', after_sat)
    for name, val in pairs:
        if name in values:
            continue
        parsed = _parse_num(val)
        values[name] = parsed if parsed is not None else val
    
    makespan = values.get("makespan", None)
    
    task_assignments = {}
    for task_id in task_pool.keys():
        for robot_name, robot_data in robots.items():
            robot_id = robot_data["id"]
            assign_var = f"assign_{task_id}_{robot_id}"
            if values.get(assign_var, 0) == 1:
                task_assignments[task_id] = robot_id
                break
    
    start_times = {}
    for task_id in task_pool.keys():
        start_var = f"s_{task_id}"
        start_times[task_id] = values.get(start_var, 0.0)
    
    schedule = {}
    for robot_name, robot_data in robots.items():
        robot_id = robot_data["id"]
        robot_tasks = []
        
        for task_id, assigned_robot in task_assignments.items():
            if assigned_robot == robot_id:
                task_data = task_pool[task_id]
                robot_tasks.append({
                    "task_id": task_id,
                    "description": task_data.get("description", task_id),
                    "start_time": start_times[task_id],
                    "duration": task_data["duration"],
                    "end_time": start_times[task_id] + task_data["duration"],
                    "location": task_data["location"],
                })
        
        robot_tasks.sort(key=lambda t: t["start_time"])
        schedule[robot_name] = robot_tasks
    
    return {
        "makespan": makespan,
        "task_assignments": task_assignments,
        "start_times": start_times,
        "schedule": schedule,
    }


def export_schedule_json(result, task_pool, robots, resources, run_dir):
    output = {
        "timestamp": time.time(),
        "makespan": result["makespan"],
        "task_assignments": result["task_assignments"],
        "schedule": result["schedule"],
        "task_pool": task_pool,
        "robots": {name: {
            "id": data["id"],
            "capabilities": data["capabilities"],
            "max_speed": data["max_speed"]
        } for name, data in robots.items()},
        "resources": resources,
    }
    
    output_path = run_dir / "schedule_heterogeneous.json"
    output_path.write_text(json.dumps(output, indent=2))
    print(f"[EXPORT] Saved -> {output_path}")
    
    return output


def print_schedule_summary(result, robots):
    print("\n" + "="*70)
    print("HETEROGENEOUS TASK ALLOCATION RESULT")
    print("="*70)
    
    makespan_val = result.get('makespan')
    if makespan_val is not None:
        try:
            if isinstance(makespan_val, str):
                if "/" in makespan_val:
                    parts = makespan_val.split("/")
                    makespan_val = float(parts[0]) / float(parts[1])
                else:
                    makespan_val = float(makespan_val)
            else:
                makespan_val = float(makespan_val)
            print(f"Makespan: {makespan_val:.2f}s")
        except (ValueError, TypeError):
            print(f"Makespan: {makespan_val}")
    else:
        print("Makespan: N/A")
    
    print("")
    
    for robot_name, tasks in result["schedule"].items():
        robot_data = robots[robot_name]
        print(f"Robot {robot_data['id']} ({robot_name}):")
        print(f"  Capabilities: {', '.join(robot_data['capabilities'])}")
        print(f"  Max Speed: {robot_data['max_speed']} m/s")
        print(f"  Tasks: {len(tasks)}")
        
        if tasks:
            for task in tasks:
                try:
                    start = float(task['start_time'])
                    end = float(task['end_time'])
                    time_str = f"[{start:.1f}s - {end:.1f}s]"
                except (ValueError, TypeError):
                    time_str = f"[{task['start_time']} - {task['end_time']}]"
                
                print(f"    {task['task_id']}: {time_str}")
        print("")


def main():
    parser = argparse.ArgumentParser(description="Heterogeneous Multi-Robot Task Scheduler")
    parser.add_argument("--config", default="config_warehouse_3x6.yaml", help="Config file")
    parser.add_argument("--timeout", type=int, default=120, help="Z3 timeout (seconds)")
    
    args = parser.parse_args()
    
    print("="*70)
    print("Heterogeneous Multi-Robot Task Scheduler (SMT)")
    print("="*70)
    print("")
    
    config = load_config(args.config)
    
    task_pool = build_task_pool(config)
    robots = build_robots(config)
    resources = build_resources(config)
    
    print("")
    
    if not validate_configuration(task_pool, robots):
        sys.exit(1)
    print("")
    
    run_dir = Path(RUNS_DIR) / time.strftime("%Y%m%d-%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    
    config_export = {
        "task_pool": task_pool,
        "robots": robots,
        "resources": resources,
    }
    (run_dir / "config.json").write_text(json.dumps(config_export, indent=2))
    
    print("[SMT] Generating SMT2 encoding...")
    smt2_path = run_dir / "heterogeneous.smt2"
    export_smt2_heterogeneous(task_pool, robots, resources, smt2_path)
    print("")
    
    print(f"[Z3] Running solver (timeout={args.timeout}s)...")
    solve_start = time.time()
    z3_ok, z3_output = run_z3_solver(smt2_path, timeout=args.timeout)
    solve_time = time.time() - solve_start
    
    (run_dir / "z3_result.txt").write_text(z3_output if z3_output else "")
    
    if not z3_ok:
        print(f"[Z3] ERROR: {z3_output}")
        print("")
        print("[RESULT] FAILED")
        return False
    
    first_line = z3_output.strip().split("\n")[0] if z3_output else ""
    print(f"[Z3] {first_line} ({solve_time:.3f}s)")
    print("")
    
    if first_line.strip() != "sat":
        print("[RESULT] UNSAT - No feasible schedule")
        return False
    
    print("[PARSE] Extracting schedule...")
    result = parse_z3_result(z3_output, task_pool, robots)
    
    if not result:
        print("[ERROR] Failed to parse Z3 output")
        return False
    
    export_schedule_json(result, task_pool, robots, resources, run_dir)
    
    print_schedule_summary(result, robots)
    
    print("="*70)
    print("STATISTICS")
    print("="*70)
    print(f"Solver time: {solve_time:.3f}s")
    
    makespan_val = result.get('makespan')
    if makespan_val is not None:
        try:
            if isinstance(makespan_val, str) and "/" in makespan_val:
                p, q = makespan_val.split("/", 1)
                makespan_val = float(p) / float(q)
            else:
                makespan_val = float(makespan_val)
            print(f"Makespan: {makespan_val:.2f}s")
        except (ValueError, TypeError):
            print(f"Makespan: {makespan_val}")
    
    print(f"Tasks assigned: {len(result['task_assignments'])}/{len(task_pool)}")
    
    print("\nLoad distribution:")
    for robot_name, tasks in result["schedule"].items():
        print(f"  {robot_name}: {len(tasks)}")
    
    print(f"\n[SUCCESS] Results saved to: {run_dir}")
    print("")
    
    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)

