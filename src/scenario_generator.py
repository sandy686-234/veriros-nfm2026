#!/usr/bin/env python3

import yaml
import subprocess
import json
import time
from pathlib import Path

def generate_scenario(n_robots, n_tasks):
    """Generate a scenario with n robots and n tasks"""
    robots = []
    capabilities = ['lift', 'transport', 'sensor']
    
    for i in range(n_robots):
        cap = capabilities[i % len(capabilities)]
        robots.append({
            'name': f'Robot_{i}',
            'id': chr(65 + i),
            'capabilities': [cap],
            'max_speed': 100000.0,
            'start_position': [0.0, 0.0]
        })
    
    tasks = []
    task_durations = [15, 18, 20, 22, 25, 30, 12, 16, 19, 23, 28, 32]
    
    for i in range(n_tasks):
        duration = task_durations[i % len(task_durations)]
        deadline = (sum(task_durations[:i+1]) // n_robots) + 50
        capability = capabilities[i % len(capabilities)]
        
        tasks.append({
            'id': f'task_{i}',
            'location': [0.0, 0.0],
            'duration': duration,
            'deadline': deadline,
            'requires_capability': capability,
            'uses_resources': []
        })
    
    scenario = {
        'scenario': {'name': f'test_{n_robots}x{n_tasks}', 'n_robots': n_robots, 'n_tasks': n_tasks},
        'robots': robots,
        'tasks': tasks,
        'resources': {}
    }
    return scenario

def run_scenario(config, scheduler_path, timeout=120):
    """Run a single scenario and return solver time and makespan"""
    config_file = '/tmp/test_config.yaml'
    with open(config_file, 'w') as f:
        yaml.dump(config, f)
    
    try:
        start = time.time()
        result = subprocess.run(
            ['python3', str(scheduler_path), '--config', config_file, '--timeout', str(timeout)],
            capture_output=True, text=True, timeout=timeout + 10
        )
        elapsed = time.time() - start
        
        output = result.stdout
        makespan = None
        solver_time = None
        
        for line in output.split('\n'):
            if 'Makespan:' in line and not 'Optimal' in line:
                try:
                    makespan = float(line.split('Makespan:')[1].strip().split('s')[0])
                except:
                    pass
            if 'Solver time:' in line:
                try:
                    solver_time = float(line.split('Solver time:')[1].strip().split('s')[0])
                except:
                    pass
        
        return {
            'success': result.returncode == 0,
            'solver_time': solver_time,
            'makespan': makespan,
            'wall_time': elapsed
        }
    except Exception as e:
        return {'success': False, 'error': str(e), 'solver_time': None, 'makespan': None}

def main():
    scheduler_path = Path('./src/gate_heterogeneous_scheduler.py')
    
    test_cases = [(2, 4), (3, 6), (4, 8), (5, 10), (6, 12)]
    results = {'timestamp': time.time(), 'test_cases': []}
    
    print("="*70)
    print("SCALABILITY TEST SUITE")
    print("="*70)
    print(f"{'Robots':<8} {'Tasks':<8} {'Solver Time (s)':<20} {'Makespan (s)':<15} {'Status':<10}")
    print("-"*70)
    
    for n_robots, n_tasks in test_cases:
        config = generate_scenario(n_robots, n_tasks)
        result = run_scenario(config, scheduler_path)
        
        results['test_cases'].append({
            'n_robots': n_robots,
            'n_tasks': n_tasks,
            'solver_time': result['solver_time'],
            'makespan': result['makespan']
        })
        
        status = 'PASS' if result['success'] else 'FAIL'
        solver_time = f"{result['solver_time']:.3f}" if result['solver_time'] else 'N/A'
        makespan = f"{result['makespan']:.1f}" if result['makespan'] else 'N/A'
        
        print(f"{n_robots:<8} {n_tasks:<8} {solver_time:<20} {makespan:<15} {status:<10}")
    
    print("="*70)
    
    output_file = Path('scalability_results.json')
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"[SUCCESS] Results saved to {output_file}\n")

if __name__ == '__main__':
    main()
