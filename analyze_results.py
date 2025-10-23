#!/usr/bin/env python3
import json
import matplotlib.pyplot as plt
import numpy as np

# Load both datasets
with open('data/metrics.json', 'r') as f:
    manual_data = json.load(f)

with open('data/metrics_auto.json', 'r') as f:
    auto_data = json.load(f)

# Extract data
manual_times = [p['time'] for p in manual_data]
manual_coverage = [p['coverage'] for p in manual_data]

auto_times = [p['time'] for p in auto_data]
auto_coverage = [p['coverage'] for p in auto_data]

# Create figure with subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

# Plot 1: Automated exploration
ax1.plot(auto_times, auto_coverage, 'b-', linewidth=2, label='Coverage')
ax1.set_xlabel('Time (s)', fontsize=11)
ax1.set_ylabel('Coverage (%)', fontsize=11)
ax1.set_title('Automated Exploration', fontsize=12, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.set_xlim(0, max(auto_times))
ax1.set_ylim(0, 100)

# Add annotations for auto
max_cov_auto = max(auto_coverage)
final_time_auto = auto_times[-1]
ax1.axhline(y=max_cov_auto, color='r', linestyle='--', alpha=0.5, linewidth=1)
ax1.text(0.05, 0.95, f'Peak: {max_cov_auto:.1f}%\nTime: {final_time_auto:.0f}s',
         transform=ax1.transAxes, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7), fontsize=9)

# Plot 2: Manual exploration
ax2.plot(manual_times, manual_coverage, 'g-', linewidth=2, label='Coverage')
ax2.set_xlabel('Time (s)', fontsize=11)
ax2.set_ylabel('Coverage (%)', fontsize=11)
ax2.set_title('Manual Exploration', fontsize=12, fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.set_xlim(0, max(manual_times))
ax2.set_ylim(0, 100)

# Add annotations for manual
max_cov_manual = max(manual_coverage)
final_time_manual = manual_times[-1]
ax2.axhline(y=max_cov_manual, color='r', linestyle='--', alpha=0.5, linewidth=1)
ax2.text(0.05, 0.95, f'Peak: {max_cov_manual:.1f}%\nTime: {final_time_manual:.0f}s',
         transform=ax2.transAxes, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7), fontsize=9)

plt.tight_layout()
plt.savefig('figures/coverage_comparison.pdf', dpi=300, bbox_inches='tight')
plt.savefig('figures/coverage_comparison.png', dpi=300, bbox_inches='tight')
print(f"Plots saved to figures/")

# Print statistics
print("\n=== Automated Exploration Statistics ===")
print(f"Duration: {final_time_auto:.1f}s")
print(f"Peak Coverage: {max_cov_auto:.1f}%")
print(f"Final Coverage: {auto_coverage[-1]:.1f}%")
print(f"Data points: {len(auto_data)}")

print("\n=== Manual Exploration Statistics ===")
print(f"Duration: {final_time_manual:.1f}s")
print(f"Peak Coverage: {max_cov_manual:.1f}%")
print(f"Final Coverage: {manual_coverage[-1]:.1f}%")
print(f"Data points: {len(manual_data)}")

# Calculate exploration rate (coverage per second)
auto_rate = max_cov_auto / final_time_auto if final_time_auto > 0 else 0
manual_rate = max_cov_manual / final_time_manual if final_time_manual > 0 else 0

print(f"\nAutomated exploration rate: {auto_rate:.3f}%/s")
print(f"Manual exploration rate: {manual_rate:.3f}%/s")
