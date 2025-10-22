#!/usr/bin/env python3
import json
import matplotlib.pyplot as plt
import os
import subprocess

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

plt.tight_layout()

# Save to /tmp as backup
plt.savefig('/tmp/exploration_plot.png', dpi=150)
print("Plot saved to /tmp/exploration_plot.png")

# Auto-detect Windows username and save to Desktop
try:
    # Get Windows username from the mounted C: drive path
    result = subprocess.run(['cmd.exe', '/c', 'echo %USERNAME%'], 
                          capture_output=True, text=True)
    windows_user = result.stdout.strip()
    
    desktop_path = f'/mnt/c/Users/{windows_user}/Desktop/exploration_plot.png'
    plt.savefig(desktop_path, dpi=150)
    print(f"Plot saved to Desktop: C:\\Users\\{windows_user}\\Desktop\\exploration_plot.png")
except Exception as e:
    print(f"Could not save to Desktop: {e}")
    print("Plot is still available at /tmp/exploration_plot.png")

plt.show()
