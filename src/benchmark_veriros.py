#!/usr/bin/env python3

import time
import json
import subprocess
import yaml
from pathlib import Path
from collections import defaultdict
from gate_heterogeneous_scheduler import (
    load_config, build_task_pool, build_robots, build_resources,
    export_smt2_heterogeneous, run_z3_solver, parse_z3_result
)

def run_edf_baseline(task_pool, robots):
    """Earliest Deadline First baseline"""
    start = time.time()
    tasks = sorted(task_pool.items(), key=lambda x: x[1]["deadline"])
    robot_available = {r["id"]: 0.0 for r in robots.values()}
    schedule = []
    deadlines_met = 0
    
    for tid, tdata in tasks:
        capable = []
        for rname, rdata in robots.items():
            req_cap = tdata.get("requires_capability")
            if req_cap is None or req_cap in rdata["capabilities"]:
                capable.append((rdata["id"], rdata, robot_available[rdata["id"]]))
        
        if not capable:
            continue
        
        rid, rdata, avail_time = min(capable, key=lambda x: x[2])
        start_time = avail_time
        end_time = start_time + tdata["duration"]
        
        if end_time <= tdata["deadline"]:
            deadlines_met += 1
        
        schedule.append((tid, rid, start_time, end_time))
        robot_available[rid] = end_time
    
    makespan = max(end for _, _, _, end in schedule) if schedule else 0
    return {
        "makespan": makespan,
        "conflicts": 0,
        "deadlines_met": deadlines_met,
        "solve_time": time.time() - start
    }

def run_priority_baseline(task_pool, robots):
    """Priority-based baseline"""
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
    robot_available = {r["id"]: 0.0 for r in robots.values()}
    schedule = []
    deadlines_met = 0
    
    for tid, tdata in tasks:
        capable = []
        for rname, rdata in robots.items():
            req_cap = tdata.get("requires_capability")
            if req_cap is None or req_cap in rdata["capabilities"]:
                capable.append((rdata["id"], rdata, robot_available[rdata["id"]]))
        
        if not capable:
            continue
        
        rid, rdata, avail_time = min(capable, key=lambda x: x[2])
        start_time = avail_time
        end_time = start_time + tdata["duration"]
        
        if end_time <= tdata["deadline"]:
            deadlines_met += 1
        
        schedule.append((tid, rid, start_time, end_time))
        robot_available[rid] = end_time
    
    makespan = max(end for _, _, _, end in schedule) if schedule else 0
    return {
        "makespan": makespan,
        "conflicts": 0,
        "deadlines_met": deadlines_met,
        "solve_time": time.time() - start
    }

def run_veriros_smt(task_pool, robots, resources, run_dir):
    """VeriROS SMT-based scheduler"""
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
        "conflicts": 0,
        "deadlines_met": deadlines_met,
        "solve_time": solve_time
    }

def run_benchmark(config_file="configs/config_warehouse_3x6.yaml"):
    """Run complete benchmark"""
    print("="*80)
    print("VeriROS-Exec Performance Benchmark")
    print("="*80)
    
    config = load_config(config_file)
    task_pool = build_task_pool(config)
    robots = build_robots(config)
    resources = build_resources(config)
    
    print(f"\nScenario: {len(robots)} Robots, {len(task_pool)} Tasks\n")
    
    run_dir = Path("benchmark_results") / time.strftime("%Y%m%d-%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    
    results = {}
    
    print("[1/3] Running VeriROS (SMT-based)...")
    veri_result = run_veriros_smt(task_pool, robots, resources, run_dir)
    results["VeriROS (SMT Opt.)"] = veri_result
    print(f"      ✓ Makespan: {veri_result['makespan']:.1f}s, Solver: {veri_result['solve_time']:.3f}s\n")
    
    print("[2/3] Running Naive EDF (Online)...")
    edf_result = run_edf_baseline(task_pool, robots)
    results["Naive EDF (Online)"] = edf_result
    print(f"      ✓ Makespan: {edf_result['makespan']:.1f}s, Deadlines: {edf_result['deadlines_met']}/{len(task_pool)}\n")
    
    print("[3/3] Running Priority-based (Fixed)...")
    priority_result = run_priority_baseline(task_pool, robots)
    results["Priority (Fixed)"] = priority_result
    print(f"      ✓ Makespan: {priority_result['makespan']:.1f}s, Deadlines: {priority_result['deadlines_met']}/{len(task_pool)}\n")
    
    print("="*80)
    print("RESULTS TABLE")
    print("="*80)
    print(f"{'Method':<25} {'Makespan (s)':<18} {'Conflicts':<12} {'Deadlines Met':<20} {'Solver Time':<15}")
    print("-"*80)
    
    for method in ["Naive EDF (Online)", "Priority (Fixed)", "VeriROS (SMT Opt.)"]:
        data = results[method]
        makespan = f"{data['makespan']:.1f}" if data['makespan'] != float('inf') else "INFEAS"
        deadlines = f"{data['deadlines_met']}/{len(task_pool)}"
        time_str = f"{data['solve_time']:.3f}s"
        print(f"{method:<25} {makespan:<18} {data['conflicts']:<12} {deadlines:<20} {time_str:<15}")
    
    (run_dir / "benchmark_results.json").write_text(json.dumps(results, indent=2, default=str))
    print(f"\n[SUCCESS] Results saved to {run_dir}/benchmark_results.json\n")
    return results

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="VeriROS-Exec Benchmark")
    parser.add_argument("--config", default="configs/config_warehouse_3x6.yaml", help="Config file")
    args = parser.parse_args()
    run_benchmark(config_file=args.config)
