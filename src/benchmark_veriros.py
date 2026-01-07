#!/usr/bin/env python3

import time
import json
import math
from pathlib import Path
from gate_heterogeneous_scheduler import (
    load_config, build_task_pool, build_robots, build_resources,
    export_smt2_heterogeneous, run_z3_solver, parse_z3_result,
    compute_travel_time
)


def run_edf_baseline(task_pool, robots, with_travel_time=True):
    """
    Earliest Deadline First baseline.
    If with_travel_time=True, considers travel time between tasks.
    """
    start = time.time()
    tasks = sorted(task_pool.items(), key=lambda x: x[1]["deadline"])
    
    # Track robot state: available time and current position
    robot_state = {}
    for rname, rdata in robots.items():
        robot_state[rdata["id"]] = {
            "available_time": 0.0,
            "position": rdata["start_position"],
            "speed": rdata["max_speed"],
            "capabilities": rdata["capabilities"],
            "name": rname
        }
    
    schedule = []
    deadlines_met = 0
    conflicts = 0
    
    for tid, tdata in tasks:
        req_cap = tdata.get("requires_capability")
        task_loc = tdata["location"]
        
        # Find capable robots
        capable = []
        for rid, state in robot_state.items():
            if req_cap is None or req_cap in state["capabilities"]:
                # Calculate when robot can start this task
                if with_travel_time:
                    travel = compute_travel_time(state["position"], task_loc, state["speed"])
                else:
                    travel = 0.0
                earliest_start = state["available_time"] + travel
                capable.append((rid, earliest_start, travel))
        
        if not capable:
            conflicts += 1
            continue
        
        # Choose robot with earliest availability
        rid, start_time, travel = min(capable, key=lambda x: x[1])
        end_time = start_time + tdata["duration"]
        
        # Check deadline
        if end_time <= tdata["deadline"]:
            deadlines_met += 1
        else:
            conflicts += 1
        
        schedule.append((tid, rid, start_time, end_time))
        
        # Update robot state
        robot_state[rid]["available_time"] = end_time
        robot_state[rid]["position"] = task_loc
    
    makespan = max(end for _, _, _, end in schedule) if schedule else 0
    return {
        "makespan": makespan,
        "conflicts": conflicts,
        "deadlines_met": deadlines_met,
        "solve_time": time.time() - start,
        "schedule": schedule
    }


def run_priority_baseline(task_pool, robots, with_travel_time=True):
    """
    Priority-based baseline: heavy_lift > sensor > transport > other
    If with_travel_time=True, considers travel time between tasks.
    """
    start = time.time()
    
    def priority(tid, tdata):
        req = tdata.get("requires_capability")
        if req == "heavy_lift":
            return 3
        elif req == "sensor":
            return 2
        elif req == "transport":
            return 1
        else:
            return 0
    
    tasks = sorted(task_pool.items(), 
                   key=lambda x: (-priority(x[0], x[1]), x[1]["deadline"]))
    
    # Track robot state
    robot_state = {}
    for rname, rdata in robots.items():
        robot_state[rdata["id"]] = {
            "available_time": 0.0,
            "position": rdata["start_position"],
            "speed": rdata["max_speed"],
            "capabilities": rdata["capabilities"],
            "name": rname
        }
    
    schedule = []
    deadlines_met = 0
    conflicts = 0
    
    for tid, tdata in tasks:
        req_cap = tdata.get("requires_capability")
        task_loc = tdata["location"]
        
        capable = []
        for rid, state in robot_state.items():
            if req_cap is None or req_cap in state["capabilities"]:
                if with_travel_time:
                    travel = compute_travel_time(state["position"], task_loc, state["speed"])
                else:
                    travel = 0.0
                earliest_start = state["available_time"] + travel
                capable.append((rid, earliest_start, travel))
        
        if not capable:
            conflicts += 1
            continue
        
        rid, start_time, travel = min(capable, key=lambda x: x[1])
        end_time = start_time + tdata["duration"]
        
        if end_time <= tdata["deadline"]:
            deadlines_met += 1
        else:
            conflicts += 1
        
        schedule.append((tid, rid, start_time, end_time))
        robot_state[rid]["available_time"] = end_time
        robot_state[rid]["position"] = task_loc
    
    makespan = max(end for _, _, _, end in schedule) if schedule else 0
    return {
        "makespan": makespan,
        "conflicts": conflicts,
        "deadlines_met": deadlines_met,
        "solve_time": time.time() - start,
        "schedule": schedule
    }


def run_veriros_smt(task_pool, robots, resources, run_dir):
    """VeriROS SMT-based optimal scheduler"""
    start = time.time()
    smt2_path = run_dir / "benchmark.smt2"
    export_smt2_heterogeneous(task_pool, robots, resources, smt2_path)
    z3_ok, z3_output = run_z3_solver(smt2_path, timeout=120)
    solve_time = time.time() - start
    
    if not z3_ok or not z3_output:
        return {"makespan": float('inf'), "conflicts": 0, "deadlines_met": 0, "solve_time": solve_time}
    
    first_line = z3_output.strip().split("\n")[0].strip() if z3_output else ""
    if first_line != "sat":
        return {"makespan": float('inf'), "conflicts": 0, "deadlines_met": 0, "solve_time": solve_time}
    
    result = parse_z3_result(z3_output, task_pool, robots)
    if not result:
        return {"makespan": float('inf'), "conflicts": 0, "deadlines_met": 0, "solve_time": solve_time}
    
    makespan = result.get("makespan", float('inf'))
    if isinstance(makespan, str):
        if "/" in makespan:
            parts = makespan.split("/")
            makespan = float(parts[0]) / float(parts[1])
        else:
            makespan = float(makespan)
    else:
        makespan = float(makespan)
    
    deadlines_met = sum(1 for tid in task_pool if 
                       result["start_times"][tid] + task_pool[tid]["duration"] 
                       <= task_pool[tid]["deadline"])
    
    return {
        "makespan": makespan,
        "conflicts": 0,  # SMT guarantees no conflicts
        "deadlines_met": deadlines_met,
        "solve_time": solve_time
    }


def run_benchmark(config_file="config_warehouse_3x6.yaml"):
    """Run complete benchmark with fair comparison"""
    
    print("="*80)
    print("VeriROS Performance Benchmark (Fair Comparison with Travel Time)")
    print("="*80)
    
    config = load_config(config_file)
    task_pool = build_task_pool(config)
    robots = build_robots(config)
    resources = build_resources(config)
    
    print(f"\nScenario: {len(robots)} Robots, {len(task_pool)} Tasks, {len(resources)} Resources\n")
    
    run_dir = Path("benchmark_results") / time.strftime("%Y%m%d-%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    
    results = {}
    
    # Run VeriROS
    print("[1/5] Running VeriROS (SMT-based, optimal)...")
    veri_result = run_veriros_smt(task_pool, robots, resources, run_dir)
    results["VeriROS (SMT)"] = veri_result
    print(f"      ✓ Makespan: {veri_result['makespan']:.1f}s, Solver: {veri_result['solve_time']:.3f}s\n")
    
    # Run EDF with travel time
    print("[2/5] Running EDF (with travel time)...")
    edf_travel = run_edf_baseline(task_pool, robots, with_travel_time=True)
    results["EDF (travel)"] = edf_travel
    print(f"      ✓ Makespan: {edf_travel['makespan']:.1f}s, Deadlines: {edf_travel['deadlines_met']}/{len(task_pool)}\n")
    
    # Run EDF without travel time (unfair baseline)
    print("[3/5] Running EDF (no travel time - unfair)...")
    edf_no_travel = run_edf_baseline(task_pool, robots, with_travel_time=False)
    results["EDF (no travel)"] = edf_no_travel
    print(f"      ✓ Makespan: {edf_no_travel['makespan']:.1f}s (unrealistic)\n")
    
    # Run Priority with travel time
    print("[4/5] Running Priority (with travel time)...")
    priority_travel = run_priority_baseline(task_pool, robots, with_travel_time=True)
    results["Priority (travel)"] = priority_travel
    print(f"      ✓ Makespan: {priority_travel['makespan']:.1f}s, Deadlines: {priority_travel['deadlines_met']}/{len(task_pool)}\n")
    
    # Run Priority without travel time
    print("[5/5] Running Priority (no travel time - unfair)...")
    priority_no_travel = run_priority_baseline(task_pool, robots, with_travel_time=False)
    results["Priority (no travel)"] = priority_no_travel
    print(f"      ✓ Makespan: {priority_no_travel['makespan']:.1f}s (unrealistic)\n")
    
    # Print results table
    print("="*80)
    print("RESULTS TABLE (Fair Comparison)")
    print("="*80)
    print(f"{'Method':<25} {'Makespan (s)':<15} {'Deadlines':<15} {'Solver Time':<15}")
    print("-"*80)
    
    for method in ["VeriROS (SMT)", "EDF (travel)", "Priority (travel)", 
                   "EDF (no travel)", "Priority (no travel)"]:
        data = results[method]
        makespan = f"{data['makespan']:.1f}" if data['makespan'] != float('inf') else "INFEAS"
        deadlines = f"{data['deadlines_met']}/{len(task_pool)}"
        time_str = f"{data['solve_time']:.3f}s"
        note = " (unrealistic)" if "no travel" in method else ""
        print(f"{method:<25} {makespan:<15} {deadlines:<15} {time_str:<15}{note}")
    
    # Calculate improvements (fair comparison)
    print("\n" + "="*80)
    print("IMPROVEMENTS (VeriROS vs. Fair Baselines)")
    print("="*80)
    
    veri = results["VeriROS (SMT)"]
    edf = results["EDF (travel)"]
    priority = results["Priority (travel)"]
    
    if veri['makespan'] != float('inf') and edf['makespan'] > 0:
        imp = (edf['makespan'] - veri['makespan']) / edf['makespan'] * 100
        print(f"vs. EDF (with travel):      {imp:+.1f}% makespan")
    
    if veri['makespan'] != float('inf') and priority['makespan'] > 0:
        imp = (priority['makespan'] - veri['makespan']) / priority['makespan'] * 100
        print(f"vs. Priority (with travel): {imp:+.1f}% makespan")
    
    # Save results
    (run_dir / "benchmark_results.json").write_text(json.dumps(results, indent=2, default=str))
    
    print(f"\n[SUCCESS] Results saved to {run_dir}/benchmark_results.json")
    print("")
    
    return results


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="VeriROS Benchmark")
    parser.add_argument("--config", default="config_warehouse_3x6.yaml", help="Config file")
    args = parser.parse_args()
    run_benchmark(config_file=args.config)