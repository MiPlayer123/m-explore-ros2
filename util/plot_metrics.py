#!/usr/bin/env python3
import json
import matplotlib.pyplot as plt

# Read the data
with open('/tmp/metrics.json', 'r') as f:
    data = json.load(f)

# Extract time and coverage
times = [point['time'] for point in data]
coverages = [point['coverage'] for point in data]

# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(times, coverages, 'b-', linewidth=2, label='Coverage %')
plt.xlabel('Time (seconds)', fontsize=12)
plt.ylabel('Coverage (%)', fontsize=12)
plt.title('Autonomous Exploration - Coverage Over Time', fontsize=14, fontweight='bold')
plt.grid(True, alpha=0.3)

# Legend in lower right corner
plt.legend(loc='lower right', fontsize=10)

# Add stats text in upper left (only max coverage)
if data:
    max_cov = max(coverages)
    stats_text = f'Max Coverage: {max_cov:.1f}%'
    plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
             fontsize=10, verticalalignment='top', 
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

# Save
plt.tight_layout()
plt.savefig('/tmp/exploration_plot.png', dpi=150)
print("Plot saved to /tmp/exploration_plot.png")

# Also copy to Desktop
try:
    plt.savefig('/mnt/c/Users/Jeff/Desktop/exploration_plot.png', dpi=150)
    print("Plot also saved to Desktop!")
except:
    print("Could not save to Desktop (that's OK)")

plt.show()
