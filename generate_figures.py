#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, Rectangle, FancyArrowPatch
import numpy as np
from pathlib import Path

Path("figures").mkdir(exist_ok=True)
plt.rcParams['font.size'] = 10
plt.rcParams['axes.linewidth'] = 1.5

# Figure 1: Architecture
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.axis('off')
ax.text(5, 9.5, 'VeriROS-Exec: System Architecture', ha='center', fontsize=14, fontweight='bold')

components = [
    (0.5, 7.5, 2, 1, 'Input:\nRobots\nTasks', '#E8F4F8'),
    (3.5, 7.5, 2, 1, 'SMT\nEncoding', '#FFF4E6'),
    (6.5, 7.5, 2, 1, 'Z3\nSolver', '#E8F8E8'),
    (3.5, 5.5, 2, 1, 'Optimal\nSchedule', '#F0E8FF'),
    (3.5, 3.5, 2, 1, 'ROS 2\nExecutor', '#E8F8F8'),
]

for x, y, w, h, label, color in components:
    box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.1",
                          edgecolor='black', facecolor=color, linewidth=1.5)
    ax.add_patch(box)
    ax.text(x + w/2, y + h/2, label, ha='center', va='center', fontsize=9)

plt.savefig('figures/figure1_architecture.pdf', dpi=300, bbox_inches='tight')
print("✓ Figure 1: Architecture")
plt.close()

# Figure 2: Scenario
fig, ax = plt.subplots(figsize=(10, 7))
ax.set_xlim(-1, 11)
ax.set_ylim(-1, 7)
ax.text(5, 6.5, '3-Robot, 6-Task Warehouse Scenario', ha='center', fontsize=14, fontweight='bold')

zones = [
    (0.5, 4.5, 2, 1, 'Shelf A', '#FFE8E8'),
    (3.5, 4.5, 2, 1, 'Shelf B', '#FFE8E8'),
    (6.5, 4.5, 2, 1, 'Packing', '#E8F4F8'),
]

for x, y, w, h, label, color in zones:
    rect = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.1",
                          edgecolor='black', facecolor=color, linewidth=2)
    ax.add_patch(rect)
    ax.text(x + w/2, y + h/2, label, ha='center', va='center', fontsize=9, fontweight='bold')

robot_positions = [(1, 3), (5, 3), (8, 3)]
robot_names = ['Forklift', 'Courier', 'Inspector']
robot_colors = ['#FF6B6B', '#4ECDC4', '#45B7D1']

for (x, y), name, color in zip(robot_positions, robot_names, robot_colors):
    circle = plt.Circle((x, y), 0.4, color=color, ec='black', linewidth=2, zorder=5)
    ax.add_patch(circle)
    ax.text(x, y, name, ha='center', va='center', fontsize=7, color='white', fontweight='bold')

ax.axis('off')
plt.savefig('figures/figure2_scenario.pdf', dpi=300, bbox_inches='tight')
print("✓ Figure 2: Scenario")
plt.close()

# Figure 3: Scalability
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

robots = [3, 4, 5, 6]
solver_times = [0.008, 0.010, 0.014, 0.023]
makespans = [50.0, 59.0, 69.0, 51.0]

ax1.plot(robots, solver_times, 'o-', linewidth=2.5, markersize=10, color='#4ECDC4')
ax1.fill_between(robots, solver_times, alpha=0.2, color='#4ECDC4')
ax1.set_xlabel('Number of Robots', fontsize=11, fontweight='bold')
ax1.set_ylabel('Solver Time (seconds)', fontsize=11, fontweight='bold')
ax1.set_title('(a) Solver Time Scalability', fontsize=12, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.set_yscale('log')

ax2.bar(robots, makespans, color='#FF6B6B', edgecolor='black', linewidth=2, width=0.6)
ax2.set_xlabel('Number of Robots', fontsize=11, fontweight='bold')
ax2.set_ylabel('Makespan (seconds)', fontsize=11, fontweight='bold')
ax2.set_title('(b) Makespan vs Robot Count', fontsize=12, fontweight='bold')
ax2.grid(True, alpha=0.3, axis='y')

for r, m in zip(robots, makespans):
    ax2.text(r, m + 2, f'{m:.1f}s', ha='center', fontsize=10, fontweight='bold')

plt.suptitle('Figure 3: Scalability Analysis', fontsize=14, fontweight='bold')
plt.savefig('figures/figure3_scalability.pdf', dpi=300, bbox_inches='tight')
print("✓ Figure 3: Scalability")
plt.close()

# Figure 4: Gantt Chart
fig, ax = plt.subplots(figsize=(12, 6))

schedule_data = {
    'Forklift': [('pickup_shelfB', 11, 29, '#FF6B6B'), ('pickup_shelfA', 29, 44, '#FF4444')],
    'Courier': [('battery_charge', 0, 30, '#4ECDC4'), ('delivery_packing', 30, 50, '#45A9B8')],
    'Inspector': [('delivery_dock', 0, 22, '#45B7D1'), ('quality_inspect', 25, 50, '#2E8BA6')],
}

for robot_idx, (robot, tasks) in enumerate(schedule_data.items()):
    for task_name, start, end, color in tasks:
        ax.barh(robot_idx, end - start, left=start, height=0.6, color=color, edgecolor='black', linewidth=1.5)
        ax.text((start + end) / 2, robot_idx, f'{task_name}\n({start}-{end}s)', 
                ha='center', va='center', fontsize=9, fontweight='bold', color='white')

ax.set_yticks(range(len(schedule_data)))
ax.set_yticklabels(list(schedule_data.keys()), fontsize=11, fontweight='bold')
ax.set_xlabel('Time (seconds)', fontsize=11, fontweight='bold')
ax.set_title('Figure 4: Optimal Schedule (Gantt Chart) - Makespan: 50.0s', fontsize=14, fontweight='bold')
ax.set_xlim(0, 160)
ax.grid(True, alpha=0.2, axis='x')

plt.savefig('figures/figure4_gantt.pdf', dpi=300, bbox_inches='tight')
print("✓ Figure 4: Gantt Chart")
plt.close()

# Figure 5: Performance Comparison
fig, ax = plt.subplots(figsize=(10, 6))

methods = ['EDF\n(Online)', 'Priority\n(Fixed)', 'VeriROS-Exec\n(SMT)']
makespans = [55.0, 55.0, 50.0]
colors = ['#FF6B6B', '#FFB84D', '#4ECDC4']

x = np.arange(len(methods))
bars = ax.bar(x, makespans, color=colors, edgecolor='black', linewidth=2)

for bar, makespan in zip(bars, makespans):
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2., height, f'{makespan:.1f}s',
            ha='center', va='bottom', fontsize=11, fontweight='bold')

ax.text(2, 52, 'Solver: 0.012s', ha='center', fontsize=10, style='italic',
        bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.3))

ax.set_ylabel('Makespan (seconds)', fontsize=12, fontweight='bold')
ax.set_title('Figure 5: Performance Comparison with Baselines', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels(methods, fontsize=11)
ax.set_ylim(0, 70)
ax.grid(True, alpha=0.3, axis='y')

plt.savefig('figures/figure5_comparison.pdf', dpi=300, bbox_inches='tight')
print("✓ Figure 5: Comparison")
plt.close()

print("\n" + "="*60)
print("✓ All figures generated successfully!")
print("="*60)
print("\nGenerated files in figures/:")
import os
for f in sorted(os.listdir('figures')):
    print(f"  - {f}")
