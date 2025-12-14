#!/usr/bin/env python3
"""
Warehouse Exploration Simulation - Final Version
Comparing single vs two robot exploration with proper baselines.
"""

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.ndimage import label, binary_dilation
import json
import os

UNKNOWN, FREE, OCCUPIED = -1, 0, 100

class WarehouseSim:
    def __init__(self, num_robots=2, coordinated=True):
        self.num_robots = num_robots
        self.coordinated = coordinated

        # Map: 250x180 cells = 12.5m x 9m = ~112m² (warehouse-like)
        self.width, self.height = 250, 180
        self.resolution = 0.05

        self.ground_truth = np.zeros((self.height, self.width), dtype=np.int8)
        self.known_map = np.full((self.height, self.width), UNKNOWN, dtype=np.int8)
        self._make_warehouse()

        # Robots start at opposite ends
        self.robots_x = [1.0, self.width * self.resolution - 1.0][:num_robots]
        self.robots_y = [1.0, 1.0][:num_robots]
        self.robot_paths = [[(self.robots_x[i], self.robots_y[i])] for i in range(num_robots)]

        self.lidar_range = int(3.5 / self.resolution)
        self.total_time = 0.0
        self.metrics_time = []
        self.metrics_cov = []

    def _make_warehouse(self):
        # Walls
        self.ground_truth[0, :] = self.ground_truth[-1, :] = OCCUPIED
        self.ground_truth[:, 0] = self.ground_truth[:, -1] = OCCUPIED

        # Warehouse shelving pattern
        for y in range(30, self.height - 30, 50):
            for x in range(35, self.width - 35, 40):
                self.ground_truth[y:y+35, x:x+12] = OCCUPIED

        # Some cross-aisles
        self.ground_truth[self.height//2-10:self.height//2+10, :] = FREE

        # Clear start areas
        self.ground_truth[5:25, 5:30] = FREE
        self.ground_truth[5:25, self.width-30:self.width-5] = FREE

    def scan(self, rx, ry):
        cx, cy = int(rx/self.resolution), int(ry/self.resolution)
        for angle in np.linspace(0, 2*np.pi, 120, endpoint=False):
            dx, dy = np.cos(angle), np.sin(angle)
            for r in range(1, self.lidar_range):
                x, y = int(cx + r*dx), int(cy + r*dy)
                if x < 0 or x >= self.width or y < 0 or y >= self.height:
                    break
                if self.known_map[y, x] == UNKNOWN:
                    self.known_map[y, x] = self.ground_truth[y, x]
                if self.ground_truth[y, x] == OCCUPIED:
                    break

    def get_frontiers(self):
        free = self.known_map == FREE
        unk = self.known_map == UNKNOWN
        adj = binary_dilation(unk, np.array([[0,1,0],[1,1,1],[0,1,0]]))
        frontier = free & adj
        labeled, n = label(frontier)
        frontiers = []
        for i in range(1, n+1):
            ys, xs = np.where(labeled == i)
            if len(xs) >= 3:
                frontiers.append({'x': np.mean(xs), 'y': np.mean(ys), 'size': len(xs), 'ig': 0})
        return frontiers

    def calc_ig(self, fx, fy):
        cx, cy = int(fx), int(fy)
        count = 0
        for angle in np.linspace(0, 2*np.pi, 36, endpoint=False):
            dx, dy = np.cos(angle), np.sin(angle)
            for r in range(1, self.lidar_range):
                x, y = int(cx + r*dx), int(cy + r*dy)
                if x < 0 or x >= self.width or y < 0 or y >= self.height:
                    break
                if self.known_map[y, x] == OCCUPIED:
                    break
                if self.known_map[y, x] == UNKNOWN:
                    count += 1
        return count

    def coverage(self):
        return 100.0 * np.sum(self.known_map == FREE) / max(1, np.sum(self.ground_truth == FREE))

    def step(self):
        # Scan from all robots
        for i in range(self.num_robots):
            self.scan(self.robots_x[i], self.robots_y[i])

        frontiers = self.get_frontiers()
        if not frontiers:
            return True

        # Calculate IG for all frontiers
        for f in frontiers:
            f['ig'] = self.calc_ig(f['x'], f['y'])

        # Assign robots to frontiers
        if self.coordinated and self.num_robots > 1 and len(frontiers) >= self.num_robots:
            # Hungarian assignment
            cost = np.zeros((self.num_robots, len(frontiers)))
            for i in range(self.num_robots):
                for j, f in enumerate(frontiers):
                    d = np.sqrt((self.robots_x[i] - f['x']*self.resolution)**2 +
                               (self.robots_y[i] - f['y']*self.resolution)**2)
                    cost[i,j] = d - 0.005 * f['ig'] * np.sqrt(f['size'])
            rows, cols = linear_sum_assignment(cost)
            assignments = [(r, frontiers[c]) for r, c in zip(rows, cols)]
        else:
            # Each robot independently picks best (distance - IG)
            assignments = []
            for i in range(self.num_robots):
                scores = [np.sqrt((self.robots_x[i] - f['x']*self.resolution)**2 +
                                 (self.robots_y[i] - f['y']*self.resolution)**2)
                         - 0.005 * f['ig'] * np.sqrt(f['size'])
                         for f in frontiers]
                assignments.append((i, frontiers[np.argmin(scores)]))

        # Move robots
        for i, f in assignments:
            tx, ty = f['x'] * self.resolution, f['y'] * self.resolution
            dx, dy = tx - self.robots_x[i], ty - self.robots_y[i]
            d = np.sqrt(dx**2 + dy**2)
            if d > 0.1:
                move = min(0.5, d)  # 0.5 m/step = 0.5 m/s at 1s timestep
                self.robots_x[i] += (dx/d) * move
                self.robots_y[i] += (dy/d) * move
                self.robot_paths[i].append((self.robots_x[i], self.robots_y[i]))

        self.total_time += 1.0
        cov = self.coverage()
        self.metrics_time.append(self.total_time)
        self.metrics_cov.append(cov)
        return cov >= 95.0

    def run(self, max_time=400):
        for i in range(self.num_robots):
            self.scan(self.robots_x[i], self.robots_y[i])

        while self.total_time < max_time:
            if self.step():
                break
        return self.total_time, self.coverage()


def main():
    print("=" * 60)
    print("WAREHOUSE EXPLORATION SIMULATION")
    print("Environment: ~112 m² warehouse (12.5m x 9m)")
    print("=" * 60)

    configs = [
        ('Single Robot (1R)', 1, False),
        ('Two Robot Uncoord (2R)', 2, False),
        ('Two Robot Coord (2R) - Ours', 2, True),
    ]

    results = {}
    all_data = {}

    for name, n, coord in configs:
        print(f"\nRunning: {name}...", flush=True)
        sim = WarehouseSim(num_robots=n, coordinated=coord)
        t, cov = sim.run(max_time=400)

        # Find milestone times
        t50 = t90 = t95 = None
        for tt, cc in zip(sim.metrics_time, sim.metrics_cov):
            if cc >= 50 and t50 is None: t50 = tt
            if cc >= 90 and t90 is None: t90 = tt
            if cc >= 95 and t95 is None: t95 = tt

        results[name] = {'t50': t50, 't90': t90, 't95': t95, 'final': cov, 'total_t': t}
        all_data[name] = {'times': sim.metrics_time, 'covs': sim.metrics_cov, 'paths': sim.robot_paths}
        print(f"  50%: {t50:.0f}s | 90%: {t90:.0f}s | 95%: {t95 or 'N/A'}s | Final: {cov:.1f}%")

    # Summary table
    baseline_90 = results['Single Robot (1R)']['t90']
    print("\n" + "=" * 60)
    print("RESULTS SUMMARY")
    print("=" * 60)
    print(f"{'Configuration':<30} {'50%':<8} {'90%':<8} {'95%':<8} {'Speedup':<10}")
    print("-" * 64)
    for name, r in results.items():
        sp = f"{baseline_90/r['t90']:.1f}x" if r['t90'] else "N/A"
        t95_str = f"{r['t95']:.0f}s" if r['t95'] else "N/A"
        print(f"{name:<30} {r['t50']:.0f}s    {r['t90']:.0f}s    {t95_str:<8} {sp:<10}")

    # JSON output
    json_out = {
        'environment': {'width_m': 12.5, 'height_m': 9.0, 'area_m2': 112},
        'results': {name: {
            'time_to_50': r['t50'],
            'time_to_90': r['t90'],
            'time_to_95': r['t95'],
            'speedup_vs_single': baseline_90/r['t90'] if r['t90'] else None,
            'final_coverage': r['final']
        } for name, r in results.items()}
    }
    with open('warehouse_results.json', 'w') as f:
        json.dump(json_out, f, indent=2)
    print("\nSaved: warehouse_results.json")

    # Generate plot
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(10, 6))
        styles = {
            'Single Robot (1R)': ('blue', '-', 'Single Robot'),
            'Two Robot Uncoord (2R)': ('orange', '--', 'Two Robot (Uncoordinated)'),
            'Two Robot Coord (2R) - Ours': ('green', '-', 'Two Robot Coordinated + IG (Ours)'),
        }

        for name, data in all_data.items():
            color, ls, lbl = styles[name]
            ax.plot(data['times'], data['covs'], color=color, linestyle=ls, linewidth=2, label=lbl)

        ax.axhline(y=90, color='red', linestyle=':', alpha=0.7, label='90% threshold')
        ax.set_xlabel('Time (seconds)', fontsize=12)
        ax.set_ylabel('Coverage (%)', fontsize=12)
        ax.set_title('Warehouse Exploration: Coverage Over Time\n(~112 m² environment)', fontsize=14)
        ax.legend(loc='lower right', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, max(max(d['times']) for d in all_data.values()) * 1.05)
        ax.set_ylim(0, 100)
        plt.tight_layout()
        plt.savefig('warehouse_comparison.png', dpi=150, bbox_inches='tight')
        plt.close()
        print("Saved: warehouse_comparison.png")

        # Also save individual exploration map for coordinated
        sim2 = WarehouseSim(num_robots=2, coordinated=True)
        sim2.run(max_time=400)

        fig2, ax2 = plt.subplots(figsize=(12, 8))
        display = np.zeros_like(sim2.known_map, dtype=float)
        display[sim2.known_map == UNKNOWN] = 0.5
        display[sim2.known_map == FREE] = 1.0
        display[sim2.known_map == OCCUPIED] = 0.0
        ax2.imshow(display, cmap='gray', origin='lower',
                   extent=[0, sim2.width*sim2.resolution, 0, sim2.height*sim2.resolution])

        colors = ['blue', 'red']
        for i in range(sim2.num_robots):
            path = np.array(sim2.robot_paths[i])
            ax2.plot(path[:, 0], path[:, 1], '-', color=colors[i], linewidth=1.5, alpha=0.8,
                    label=f'Robot {i+1}')
            ax2.plot(sim2.robots_x[i], sim2.robots_y[i], 'o', color=colors[i], markersize=12)

        ax2.set_xlabel('X (meters)', fontsize=12)
        ax2.set_ylabel('Y (meters)', fontsize=12)
        ax2.set_title(f'Warehouse Exploration - Coordinated 2-Robot\nFinal Coverage: {sim2.coverage():.1f}%', fontsize=14)
        ax2.legend(loc='upper right')
        ax2.set_aspect('equal')
        plt.tight_layout()
        plt.savefig('warehouse_exploration_map.png', dpi=150, bbox_inches='tight')
        plt.close()
        print("Saved: warehouse_exploration_map.png")

    except Exception as e:
        print(f"Plot error: {e}")

    return results


if __name__ == "__main__":
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    main()
