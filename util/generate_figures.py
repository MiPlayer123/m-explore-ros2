#!/usr/bin/env python3
"""
Generate publication-quality figures for M2 report
"""

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

# Set publication style
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 10
plt.rcParams['axes.labelsize'] = 11
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['legend.fontsize'] = 9
plt.rcParams['xtick.labelsize'] = 9
plt.rcParams['ytick.labelsize'] = 9
plt.rcParams['figure.dpi'] = 300

sns.set_palette("colorblind")

def figure_cost_matrix_heatmap():
    """Figure: Example cost matrix for 2 robots, 6 frontiers"""
    # Synthetic cost matrix
    # Rows = robots, Cols = frontiers
    # Cost = distance - IG*5 + penalties
    cost_matrix = np.array([
        [2.3, 5.1, 3.8, 7.2, 4.5, 6.1],  # Robot 1
        [6.8, 3.2, 7.5, 2.9, 5.7, 4.3],  # Robot 2
    ])

    fig, ax = plt.subplots(figsize=(6, 2.5))

    im = ax.imshow(cost_matrix, cmap='RdYlGn_r', aspect='auto', vmin=0, vmax=8)

    # Add text annotations
    for i in range(2):
        for j in range(6):
            text = ax.text(j, i, f'{cost_matrix[i, j]:.1f}',
                          ha="center", va="center", color="black", fontsize=9)

    ax.set_xticks(np.arange(6))
    ax.set_yticks(np.arange(2))
    ax.set_xticklabels([f'F{i+1}' for i in range(6)])
    ax.set_yticklabels(['Robot 1', 'Robot 2'])
    ax.set_xlabel('Frontier ID')
    ax.set_ylabel('Robot')
    ax.set_title('Cost Matrix Example (Lower = Better Assignment)')

    # Add colorbar
    cbar = plt.colorbar(im, ax=ax, label='Assignment Cost')

    # Mark optimal assignment
    optimal = [(0, 0), (1, 3)]  # Hungarian solution
    for (i, j) in optimal:
        rect = plt.Rectangle((j-0.45, i-0.45), 0.9, 0.9,
                            fill=False, edgecolor='blue', linewidth=3)
        ax.add_patch(rect)

    plt.tight_layout()
    plt.savefig('../latex_report/figures/cost_matrix.pdf', bbox_inches='tight')
    plt.savefig('../latex_report/figures/cost_matrix.png', bbox_inches='tight', dpi=300)
    print("✓ Generated cost_matrix.pdf/png")

def figure_ig_distribution():
    """Figure: Information gain distribution across frontier types"""
    # Synthetic IG data for different frontier scenarios
    np.random.seed(42)

    scenarios = ['Open\nSpace', 'Doorway', 'Corridor', 'Occluded\n(obstacles)']
    ig_data = [
        np.random.normal(120, 15, 30),  # Open space: high IG
        np.random.normal(95, 20, 30),   # Doorway: medium-high IG
        np.random.normal(65, 12, 30),   # Corridor: medium IG
        np.random.normal(35, 18, 30),   # Occluded: low IG
    ]

    fig, ax = plt.subplots(figsize=(6, 3))

    bp = ax.boxplot(ig_data, labels=scenarios, patch_artist=True,
                    boxprops=dict(facecolor='lightblue', alpha=0.7),
                    medianprops=dict(color='red', linewidth=2))

    ax.set_ylabel('Information Gain (unknown cells)')
    ax.set_xlabel('Frontier Type')
    ax.set_title('Information Gain Distribution by Frontier Context')
    ax.grid(True, alpha=0.3, axis='y')
    ax.set_ylim(0, 160)

    plt.tight_layout()
    plt.savefig('../latex_report/figures/ig_distribution.pdf', bbox_inches='tight')
    plt.savefig('../latex_report/figures/ig_distribution.png', bbox_inches='tight', dpi=300)
    print("✓ Generated ig_distribution.pdf/png")

def figure_redundancy_projection():
    """Figure: Projected redundancy vs coordination parameters"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7, 2.5))

    # IG weight sensitivity
    w_ig = np.array([1, 3, 5, 7, 9])
    redundancy_ig = 30 - 2.5 * np.log(w_ig)  # Analytical model

    ax1.plot(w_ig, redundancy_ig, 'o-', linewidth=2, markersize=8, color='#1f77b4')
    ax1.axhline(y=24, color='red', linestyle='--', linewidth=1.5, label='Target (24%)')
    ax1.axhline(y=30, color='gray', linestyle=':', linewidth=1.5, label='Baseline (30%)')
    ax1.axvline(x=5, color='green', linestyle='-.', linewidth=1.5, alpha=0.5, label='Default')
    ax1.set_xlabel('Information Gain Weight ($w_{IG}$)')
    ax1.set_ylabel('Projected Redundancy (%)')
    ax1.set_title('IG Weight Sensitivity')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=8)
    ax1.set_ylim(20, 32)

    # Cross-robot penalty sensitivity
    w_c = np.array([0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0])
    redundancy_c = 30 * np.exp(-0.3 * w_c)  # Exponential decay model

    ax2.plot(w_c, redundancy_c, 's-', linewidth=2, markersize=8, color='#ff7f0e')
    ax2.axhline(y=24, color='red', linestyle='--', linewidth=1.5, label='Target (24%)')
    ax2.axhline(y=30, color='gray', linestyle=':', linewidth=1.5, label='Baseline (30%)')
    ax2.axvline(x=2.0, color='green', linestyle='-.', linewidth=1.5, alpha=0.5, label='Default')
    ax2.set_xlabel('Cross-Robot Penalty ($w_c$)')
    ax2.set_ylabel('Projected Redundancy (%)')
    ax2.set_title('Cross-Robot Penalty Sensitivity')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=8)
    ax2.set_ylim(20, 32)

    plt.tight_layout()
    plt.savefig('../latex_report/figures/redundancy_sensitivity.pdf', bbox_inches='tight')
    plt.savefig('../latex_report/figures/redundancy_sensitivity.png', bbox_inches='tight', dpi=300)
    print("✓ Generated redundancy_sensitivity.pdf/png")

def figure_coverage_projection():
    """Figure: Projected coverage trajectories for all configurations"""
    time = np.linspace(0, 300, 100)

    # M1 baseline (single robot)
    cov_1r = 74.2 * (1 - np.exp(-time / 60))

    # Uncoordinated (2 robots, 30% redundancy)
    eff_uncoor = 1.4  # 2 * 0.7
    cov_uncoor = 74.2 * (1 - np.exp(-eff_uncoor * time / 60))

    # Coordinated (2 robots, 24% redundancy)
    eff_coord = 1.52  # 2 * 0.76
    cov_coord = 74.2 * (1 - np.exp(-eff_coord * time / 60))

    fig, ax = plt.subplots(figsize=(6, 3.5))

    ax.plot(time, cov_1r, '--', linewidth=2, label='M1 Baseline (1R)', color='gray')
    ax.plot(time, cov_uncoor, '-', linewidth=2.5, label='Uncoordinated (2R, 30% redun.)', color='#ff7f0e')
    ax.plot(time, cov_coord, '-', linewidth=2.5, label='Coordinated (2R, 24% redun.)', color='#2ca02c')

    # Mark 50% and 90% milestones
    ax.axhline(y=50, color='red', linestyle=':', linewidth=1, alpha=0.5)
    ax.axhline(y=74.2, color='red', linestyle=':', linewidth=1, alpha=0.5)
    ax.text(10, 52, '50%', fontsize=8, color='red')
    ax.text(10, 76, 'Saturation (74.2%)', fontsize=8, color='red')

    ax.set_xlabel('Time (seconds)')
    ax.set_ylabel('Coverage (%)')
    ax.set_title('Projected Coverage Trajectories')
    ax.legend(loc='lower right', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 300)
    ax.set_ylim(0, 80)

    plt.tight_layout()
    plt.savefig('../latex_report/figures/coverage_projection.pdf', bbox_inches='tight')
    plt.savefig('../latex_report/figures/coverage_projection.png', bbox_inches='tight', dpi=300)
    print("✓ Generated coverage_projection.pdf/png")

def figure_architecture_detailed():
    """Generate detailed architecture diagram as text figure"""
    # This would ideally be done in a tool like draw.io or TikZ
    # For now, create a placeholder text version
    print("⚠ Architecture diagram best created with draw.io or TikZ")
    print("  See ARCHITECTURE_DIAGRAM.txt for text-based template")

if __name__ == '__main__':
    import os
    os.makedirs('../latex_report/figures', exist_ok=True)

    print("Generating publication-quality figures for M2 report...\n")

    figure_cost_matrix_heatmap()
    figure_ig_distribution()
    figure_redundancy_projection()
    figure_coverage_projection()

    print("\n✓ All figures generated successfully!")
    print("  Location: latex_report/figures/")
    print("  Formats: PDF (LaTeX) + PNG (preview)")
