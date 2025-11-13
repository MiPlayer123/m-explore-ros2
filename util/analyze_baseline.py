#!/usr/bin/env python3
"""
Baseline Performance Analysis for M-Explore ROS2
Analyzes existing single-robot exploration data to establish baseline metrics
for comparison with multi-robot coordinated exploration.

Author: Milestone 2 Team
Date: November 2024
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

class BaselineAnalyzer:
    """Analyzes single-robot exploration metrics to establish baseline performance."""

    def __init__(self, metrics_file):
        """Load metrics data from JSON file."""
        self.metrics_file = Path(metrics_file)
        with open(self.metrics_file, 'r') as f:
            self.data = json.load(f)

        self.times = np.array([d['time'] for d in self.data])
        self.coverage = np.array([d['coverage'] for d in self.data])

    def calculate_key_metrics(self):
        """Calculate key performance metrics for baseline."""
        metrics = {}

        # Final coverage and time
        metrics['final_coverage'] = self.coverage[-1]
        metrics['total_time'] = self.times[-1]

        # Time to coverage milestones
        metrics['time_to_50'] = self._time_to_coverage(50.0)
        metrics['time_to_90'] = self._time_to_coverage(90.0)
        metrics['time_to_95'] = self._time_to_coverage(95.0)

        # Coverage rate (% per second)
        active_time = self.times[self.coverage > 10]  # After initialization
        active_coverage = self.coverage[self.coverage > 10]
        if len(active_coverage) > 1:
            coverage_gain = active_coverage[-1] - active_coverage[0]
            time_span = active_time[-1] - active_time[0]
            metrics['coverage_rate'] = coverage_gain / time_span if time_span > 0 else 0
        else:
            metrics['coverage_rate'] = 0

        # Initialization delay (time to reach 10% coverage)
        init_idx = np.where(self.coverage >= 10.0)[0]
        metrics['init_delay'] = self.times[init_idx[0]] if len(init_idx) > 0 else self.times[0]

        # Plateau detection (coverage stops increasing)
        plateau_threshold = 0.1  # Less than 0.1% change
        coverage_diff = np.diff(self.coverage)
        plateau_idx = np.where(coverage_diff < plateau_threshold)[0]
        if len(plateau_idx) > 5:  # At least 5 consecutive small changes
            metrics['plateau_start'] = self.times[plateau_idx[0]]
        else:
            metrics['plateau_start'] = None

        return metrics

    def _time_to_coverage(self, target_coverage):
        """Find time required to reach target coverage percentage."""
        idx = np.where(self.coverage >= target_coverage)[0]
        if len(idx) > 0:
            return self.times[idx[0]]
        else:
            return None  # Never reached

    def estimate_multirobot_baseline(self, num_robots=2):
        """
        Estimate multi-robot baseline performance assuming naive coordination.
        Assumes worst-case redundancy due to lack of coordination.
        """
        single_metrics = self.calculate_key_metrics()
        multi_metrics = {}

        # Assumptions for naive multi-robot (nearest-frontier per robot):
        # - Robots work independently
        # - 30% redundant exploration (overlap)
        # - Initialization delays are synchronized
        # - Coverage is additive minus redundancy

        redundancy_factor = 0.30  # 30% overlap (typical for uncoordinated)

        # Effective coverage rate = num_robots * single_rate * (1 - redundancy)
        multi_metrics['num_robots'] = num_robots
        multi_metrics['redundancy_assumed'] = redundancy_factor * 100
        multi_metrics['effective_robots'] = num_robots * (1 - redundancy_factor)

        # Time to coverage milestones (with redundancy penalty)
        single_rate = single_metrics['coverage_rate']
        multi_rate = single_rate * multi_metrics['effective_robots']

        # Estimate time to reach targets
        target_coverages = [50.0, 90.0, 95.0]
        for target in target_coverages:
            if single_rate > 0:
                est_time = (target / multi_rate) + single_metrics['init_delay']
                multi_metrics[f'estimated_time_to_{int(target)}'] = est_time
            else:
                multi_metrics[f'estimated_time_to_{int(target)}'] = None

        # Target for coordinated exploration (20% reduction in redundancy)
        target_redundancy = redundancy_factor * 0.8  # 20% reduction → 24% overlap
        multi_metrics['target_redundancy'] = target_redundancy * 100
        multi_metrics['target_effective_robots'] = num_robots * (1 - target_redundancy)

        # Estimated improvement with coordination
        improvement_factor = multi_metrics['target_effective_robots'] / multi_metrics['effective_robots']
        multi_metrics['expected_speedup'] = improvement_factor

        for target in target_coverages:
            baseline_time = multi_metrics[f'estimated_time_to_{int(target)}']
            if baseline_time:
                improved_time = baseline_time / improvement_factor
                multi_metrics[f'target_time_to_{int(target)}'] = improved_time
                multi_metrics[f'time_savings_{int(target)}'] = baseline_time - improved_time

        return multi_metrics

    def print_report(self):
        """Print comprehensive baseline analysis report."""
        single_metrics = self.calculate_key_metrics()
        multi_metrics = self.estimate_multirobot_baseline(num_robots=2)

        print("=" * 80)
        print("BASELINE PERFORMANCE ANALYSIS")
        print("=" * 80)
        print(f"\nData source: {self.metrics_file.name}")
        print(f"Total data points: {len(self.data)}")

        print("\n" + "-" * 80)
        print("SINGLE-ROBOT BASELINE METRICS")
        print("-" * 80)
        print(f"Final Coverage:           {single_metrics['final_coverage']:.1f}%")
        print(f"Total Exploration Time:   {single_metrics['total_time']:.1f}s")
        print(f"Initialization Delay:     {single_metrics['init_delay']:.1f}s")
        print(f"Coverage Rate:            {single_metrics['coverage_rate']:.3f}% per second")
        print(f"\nTime to Coverage Milestones:")
        print(f"  50% coverage:           {single_metrics['time_to_50']:.1f}s" if single_metrics['time_to_50'] else "  50% coverage:           Not reached")
        print(f"  90% coverage:           {single_metrics['time_to_90']:.1f}s" if single_metrics['time_to_90'] else "  90% coverage:           Not reached")
        print(f"  95% coverage:           {single_metrics['time_to_95']:.1f}s" if single_metrics['time_to_95'] else "  95% coverage:           Not reached")

        if single_metrics['plateau_start']:
            print(f"\nPlateau Start:            {single_metrics['plateau_start']:.1f}s")
            print(f"  (Coverage stagnated at ~{single_metrics['final_coverage']:.1f}%)")

        print("\n" + "-" * 80)
        print("MULTI-ROBOT BASELINE ESTIMATES (2 Robots, Uncoordinated)")
        print("-" * 80)
        print(f"Assumed Redundancy:       {multi_metrics['redundancy_assumed']:.1f}%")
        print(f"Effective Robot Count:    {multi_metrics['effective_robots']:.2f}")
        print(f"\nEstimated Time to Coverage:")
        print(f"  50% coverage:           {multi_metrics['estimated_time_to_50']:.1f}s" if multi_metrics['estimated_time_to_50'] else "  50% coverage:           Cannot estimate")
        print(f"  90% coverage:           {multi_metrics['estimated_time_to_90']:.1f}s" if multi_metrics['estimated_time_to_90'] else "  90% coverage:           Cannot estimate")

        print("\n" + "-" * 80)
        print("MILESTONE 2 TARGETS (With Coordinated Exploration)")
        print("-" * 80)
        print(f"Target Redundancy:        {multi_metrics['target_redundancy']:.1f}% (20% reduction)")
        print(f"Target Effective Robots:  {multi_metrics['target_effective_robots']:.2f}")
        print(f"Expected Speedup:         {multi_metrics['expected_speedup']:.2f}×")
        print(f"\nTarget Time to Coverage:")
        print(f"  50% coverage:           {multi_metrics['target_time_to_50']:.1f}s" if multi_metrics['target_time_to_50'] else "  50% coverage:           Cannot estimate")
        print(f"  90% coverage:           {multi_metrics['target_time_to_90']:.1f}s" if multi_metrics['target_time_to_90'] else "  90% coverage:           Cannot estimate")
        print(f"\nExpected Time Savings:")
        print(f"  At 50% coverage:        {multi_metrics['time_savings_50']:.1f}s" if multi_metrics.get('time_savings_50') else "  At 50% coverage:        N/A")
        print(f"  At 90% coverage:        {multi_metrics['time_savings_90']:.1f}s" if multi_metrics.get('time_savings_90') else "  At 90% coverage:        N/A")

        print("\n" + "=" * 80)
        print("SUCCESS CRITERIA FOR MILESTONE 2")
        print("=" * 80)
        print("1. Redundant exploration < 24% (vs 30% baseline)")
        print("2. Time to 90% coverage competitive or better than estimated baseline")
        print("3. Statistical significance (p < 0.05) over 10+ runs")
        print("4. Coverage plateau at ≥ 74% (match or exceed single-robot)")
        print("=" * 80 + "\n")

        return single_metrics, multi_metrics

    def plot_coverage_curve(self, save_path=None):
        """Plot coverage over time curve."""
        fig, ax = plt.subplots(figsize=(10, 6))

        ax.plot(self.times, self.coverage, 'b-', linewidth=2, label='Actual Coverage')

        # Mark milestones
        milestones = [50, 90]
        for milestone in milestones:
            time_to_milestone = self._time_to_coverage(milestone)
            if time_to_milestone:
                ax.axhline(y=milestone, color='r', linestyle='--', alpha=0.3)
                ax.axvline(x=time_to_milestone, color='r', linestyle='--', alpha=0.3)
                ax.plot(time_to_milestone, milestone, 'ro', markersize=8)
                ax.annotate(f'{milestone}% @ {time_to_milestone:.1f}s',
                           xy=(time_to_milestone, milestone),
                           xytext=(10, 10), textcoords='offset points',
                           fontsize=10, ha='left')

        ax.set_xlabel('Time (seconds)', fontsize=12)
        ax.set_ylabel('Coverage (%)', fontsize=12)
        ax.set_title('Single-Robot Exploration Baseline\nTurtleBot3 with Frontier-Based Exploration', fontsize=14)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=11)
        ax.set_xlim(0, self.times[-1] * 1.05)
        ax.set_ylim(0, 100)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Plot saved to: {save_path}")
        else:
            plt.savefig('/tmp/baseline_coverage.png', dpi=300, bbox_inches='tight')
            print("Plot saved to: /tmp/baseline_coverage.png")

        return fig, ax


def main():
    """Main analysis routine."""
    # Analyze automated run data
    data_dir = Path(__file__).parent.parent / 'data'
    metrics_file = data_dir / 'metrics_auto.json'

    if not metrics_file.exists():
        print(f"Error: Metrics file not found: {metrics_file}")
        print("Please ensure data/metrics_auto.json exists.")
        return

    print("Loading baseline data...")
    analyzer = BaselineAnalyzer(metrics_file)

    print("Calculating metrics...\n")
    single_metrics, multi_metrics = analyzer.print_report()

    print("Generating plot...")
    analyzer.plot_coverage_curve()

    # Save metrics to JSON for later use
    output_file = data_dir / 'baseline_analysis.json'
    with open(output_file, 'w') as f:
        json.dump({
            'single_robot': single_metrics,
            'multi_robot_estimates': multi_metrics,
            'source_file': str(metrics_file.name)
        }, f, indent=2, default=str)

    print(f"\nBaseline analysis saved to: {output_file}")
    print("\nNext steps:")
    print("1. Implement information gain calculation")
    print("2. Deploy coordinator with Hungarian algorithm")
    print("3. Run 2-robot coordinated exploration")
    print("4. Compare against these baseline estimates")


if __name__ == '__main__':
    main()
