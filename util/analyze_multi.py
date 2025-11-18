#!/usr/bin/env python3
"""
Coverage Comparison Analysis
Compares single robot vs multi-robot (Hungarian) exploration coverage over time.
Uses metrics_*.json files from ../data directory.

Script location: util/analyze_coverage.py
Data location: data/ (sibling to util/)
"""

import json
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import glob
import os
import sys

def load_metrics_file(filepath):
    """Load a metrics JSON file."""
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        return data
    except Exception as e:
        print(f"Error loading {filepath}: {e}")
        return None

def extract_coverage_timeseries(data):
    """Extract time and coverage arrays from metrics data."""
    if not data or 'data_points' not in data:
        return None, None
    
    times = []
    coverages = []
    
    for point in data['data_points']:
        if 'time' in point and 'coverage' in point:
            times.append(point['time'])
            coverages.append(point['coverage'])
    
    return np.array(times), np.array(coverages)

def find_latest_metrics_files(data_dir):
    """Find the most recent metrics file for single and hungarian modes."""
    modes = ['single', 'hungarian']
    latest_files = {}
    
    for mode in modes:
        pattern = str(data_dir / f'metrics_{mode}*.json')
        files = glob.glob(pattern)
        
        if files:
            # Get the most recent file (highest timestamp)
            latest_files[mode] = max(files)
            print(f"Found {mode}: {Path(latest_files[mode]).name}")
        else:
            print(f"Warning: No files found for mode '{mode}'")
    
    return latest_files

def plot_coverage_comparison(metrics_data):
    """Create coverage comparison plot."""
    plt.figure(figsize=(12, 8))
    
    colors = {
        'single': '#2E86AB',
        'hungarian': '#F18F01'
    }
    
    labels = {
        'single': 'Single Robot',
        'hungarian': 'Multi-Robot (Hungarian)'
    }
    
    max_time = 0
    max_coverage = 0
    
    for mode, data in metrics_data.items():
        times, coverages = extract_coverage_timeseries(data)
        
        if times is not None and len(times) > 0:
            plt.plot(times, coverages, 
                    label=labels.get(mode, mode),
                    color=colors.get(mode, 'gray'),
                    linewidth=2.5,
                    marker='o',
                    markersize=4,
                    markevery=max(1, len(times)//20))  # Show ~20 markers
            
            max_time = max(max_time, times[-1])
            max_coverage = max(max_coverage, coverages[-1])
            
            # Print final statistics
            print(f"\n{labels.get(mode, mode)}:")
            print(f"  Final Coverage: {coverages[-1]:.2f}%")
            print(f"  Time to 50%: {times[coverages >= 50][0]:.1f}s" if any(coverages >= 50) else "  Time to 50%: Not reached")
            print(f"  Time to 80%: {times[coverages >= 80][0]:.1f}s" if any(coverages >= 80) else "  Time to 80%: Not reached")
            print(f"  Exploration Rate: {coverages[-1]/times[-1]:.3f}%/s")
    
    plt.xlabel('Time (seconds)', fontsize=14, fontweight='bold')
    plt.ylabel('Coverage (%)', fontsize=14, fontweight='bold')
    plt.title('Exploration Coverage Comparison: Single vs Multi-Robot (Hungarian)', 
              fontsize=16, fontweight='bold', pad=20)
    plt.legend(fontsize=12, loc='lower right', framealpha=0.95)
    plt.grid(True, alpha=0.3, linestyle='--')
    plt.xlim(0, max_time * 1.05)
    plt.ylim(0, min(100, max_coverage * 1.1))
    
    # Add milestone lines
    for milestone in [25, 50, 75, 100]:
        if milestone <= max_coverage * 1.1:
            plt.axhline(y=milestone, color='gray', linestyle=':', alpha=0.3, linewidth=1)
    
    plt.tight_layout()
    
    return plt

def print_summary_statistics(metrics_data):
    """Print detailed comparison statistics."""
    print("\n" + "="*70)
    print("EXPLORATION PERFORMANCE SUMMARY")
    print("="*70)
    
    for mode in ['single', 'hungarian']:
        if mode not in metrics_data:
            continue
            
        data = metrics_data[mode]
        if not data or 'data_points' not in data:
            continue
            
        times, coverages = extract_coverage_timeseries(data)
        
        if times is None or len(times) == 0:
            continue
        
        mode_label = {'single': 'Single Robot', 
                     'hungarian': 'Multi-Robot (Hungarian)'}.get(mode, mode)
        
        print(f"\n{mode_label}:")
        print(f"  Duration: {times[-1]:.1f} seconds")
        print(f"  Final Coverage: {coverages[-1]:.2f}%")
        print(f"  Average Rate: {coverages[-1]/times[-1]:.3f}%/s")
        
        # Calculate efficiency metrics
        if len(coverages) > 1:
            avg_increase = np.mean(np.diff(coverages))
            print(f"  Avg Coverage Increase/Sample: {avg_increase:.3f}%")
        
        # Time to milestones
        milestones = [25, 50, 75, 90]
        for m in milestones:
            if any(coverages >= m):
                time_to_milestone = times[coverages >= m][0]
                print(f"  Time to {m}%: {time_to_milestone:.1f}s")
        
        # Redundancy info for multi-robot
        if mode == 'hungarian' and 'data_points' in data:
            redundancies = [p.get('redundancy', 0) for p in data['data_points'] if 'redundancy' in p]
            if redundancies:
                avg_redundancy = np.mean(redundancies)
                final_redundancy = redundancies[-1]
                print(f"  Average Redundancy: {avg_redundancy:.2f}%")
                print(f"  Final Redundancy: {final_redundancy:.2f}%")
    
    print("\n" + "="*70)

def main():
    # Get script directory (util/)
    script_dir = Path(__file__).parent.resolve()
    
    # Get parent directory (m-explore-ros2/)
    parent_dir = script_dir.parent
    
    # Data directory is sibling to util/
    data_dir = parent_dir / 'data'
    
    print("="*70)
    print("EXPLORATION COVERAGE ANALYSIS")
    print("="*70)
    print(f"Script location: {script_dir}")
    print(f"Data directory: {data_dir}")
    
    if not data_dir.exists():
        print(f"\nError: Data directory not found!")
        print(f"Expected: {data_dir}")
        print(f"Please create it or check the path.")
        return
    
    print()
    
    # Find latest metrics files
    latest_files = find_latest_metrics_files(data_dir)
    
    if not latest_files:
        print("Error: No metrics files found!")
        print(f"Looking for: metrics_single*.json and metrics_hungarian*.json")
        print(f"In directory: {data_dir}")
        return
    
    # Load data
    metrics_data = {}
    for mode, filepath in latest_files.items():
        data = load_metrics_file(filepath)
        if data:
            metrics_data[mode] = data
    
    if not metrics_data:
        print("Error: No valid data loaded!")
        return
    
    # Create plot
    print("\nGenerating comparison plot...")
    plt = plot_coverage_comparison(metrics_data)
    
    # Save plot
    output_file = data_dir / 'coverage_comparison.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")
    
    # Print statistics
    print_summary_statistics(metrics_data)
    
    # Show plot
    print("\nDisplaying plot (close window to exit)...")
    plt.show()

if __name__ == '__main__':
    main()
