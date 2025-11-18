#!/usr/bin/env python3
"""
Multi-Mode Exploration Metrics Collection
Collects coverage, timing, and redundancy metrics for different exploration strategies.

Usage:
  python3 collect_exploration_metrics.py --mode single
  python3 collect_exploration_metrics.py --mode frontier
  python3 collect_exploration_metrics.py --mode hungarian
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from explore_msgs.msg import FrontierArray
import numpy as np
import json
import time
import argparse
from pathlib import Path
from collections import defaultdict


class ExplorationMetricsCollector(Node):
    """Collects comprehensive exploration metrics for comparison."""
    
    def __init__(self, mode='single'):
        super().__init__('metrics_collector')
        
        self.mode = mode  # 'single', 'frontier', 'hungarian'
        self.start_time = time.time()
        self.data_points = []
        self.coverage_timeseries = []
        
        # Robot tracking
        if mode == 'single':
            self.robot_ids = ['']  # Single robot, no namespace
        else:
            self.robot_ids = ['robot1', 'robot2']  # Multi-robot
        
        # Coverage tracking per robot
        self.robot_maps = {rid: None for rid in self.robot_ids}
        self.robot_explored_cells = {rid: set() for rid in self.robot_ids}
        self.last_update_time = {rid: 0 for rid in self.robot_ids}
        
        # Frontier tracking (multi-robot only)
        self.frontier_counts = {rid: [] for rid in self.robot_ids}
        self.assignments = []
        
        # Merged map (for multi-robot)
        self.merged_map = None
        self.total_explored_cells = set()
        
        # Subscribe to maps
        for robot_id in self.robot_ids:
            topic = f'/{robot_id}/map' if robot_id else '/map'
            self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, rid=robot_id: self.map_callback(msg, rid),
                10
            )
            self.get_logger().info(f'Subscribed to {topic}')
        
        # Subscribe to merged map (multi-robot)
        if mode != 'single':
            self.create_subscription(
                OccupancyGrid,
                '/map',  # Merged map from map_merge
                self.merged_map_callback,
                10
            )
            self.get_logger().info('Subscribed to merged /map')
        
        # Subscribe to frontiers (multi-robot)
        if mode != 'single':
            for robot_id in self.robot_ids:
                self.create_subscription(
                    FrontierArray,
                    f'/{robot_id}/explore/frontiers_array',
                    lambda msg, rid=robot_id: self.frontier_callback(msg, rid),
                    10
                )
        
        # Timer for periodic comprehensive metrics
        self.create_timer(1.0, self.collect_metrics)
        
        # Timer for 1s coverage logging
        self.create_timer(1.0, self.log_coverage_timeseries)
        
        # Output files - save to project data directory
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        
        # Get project data directory
        data_dir = Path.home() / 'ros2_ws' / 'src' / 'm-explore-ros2' / 'data'
        data_dir.mkdir(parents=True, exist_ok=True)
        
        self.output_file = data_dir / f'metrics_{mode}_{timestamp}.json'
        self.coverage_file = data_dir / f'coverage_{mode}_{timestamp}.json'
        
        self.get_logger().info(f'Metrics collector started in {mode} mode')
        self.get_logger().info(f'Comprehensive metrics: {self.output_file}')
        self.get_logger().info(f'Coverage timeseries: {self.coverage_file}')
    
    def map_callback(self, msg, robot_id):
        """Process individual robot map updates."""
        self.robot_maps[robot_id] = msg
        self.last_update_time[robot_id] = time.time()
        
        # Extract explored cells for this robot
        data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        
        # Cells are explored if they're known (not -1 unknown)
        explored_mask = data != -1
        explored_indices = np.argwhere(explored_mask)
        
        # Convert to set of (row, col) tuples for this robot
        new_cells = set()
        for row, col in explored_indices:
            # Convert to global coordinates using map origin
            x = msg.info.origin.position.x + col * msg.info.resolution
            y = msg.info.origin.position.y + row * msg.info.resolution
            # Discretize to grid cells (10cm resolution for tracking)
            cell_x = int(x * 10)
            cell_y = int(y * 10)
            new_cells.add((cell_x, cell_y))
        
        self.robot_explored_cells[robot_id] = new_cells
    
    def merged_map_callback(self, msg):
        """Process merged map (multi-robot only)."""
        self.merged_map = msg
    
    def frontier_callback(self, msg, robot_id):
        """Track frontier counts per robot."""
        self.frontier_counts[robot_id].append({
            'time': time.time() - self.start_time,
            'count': len(msg.frontiers),
            'total_ig': sum(f.information_gain for f in msg.frontiers)
        })
    
    def calculate_coverage_accurate(self, occupancy_grid):
        """
        Calculate coverage using bounding box method.
        Coverage = known cells / (bounding box of known cells)
        This excludes the huge unknown padding that map_merge adds.
        """
        if occupancy_grid is None:
            return 0.0
        
        try:
            data = np.array(occupancy_grid.data)
            h = occupancy_grid.info.height
            w = occupancy_grid.info.width
            
            if len(data) == 0 or h == 0 or w == 0:
                return 0.0
            
            # Reshape to 2D grid
            grid = data.reshape((h, w))
            
            # Find all known cells (not -1)
            known_mask = grid != -1
            known_count = np.sum(known_mask)
            
            if known_count == 0:
                return 0.0
            
            # Find bounding box of known cells
            rows = np.any(known_mask, axis=1)
            cols = np.any(known_mask, axis=0)
            
            if not rows.any() or not cols.any():
                return 0.0
            
            row_indices = np.where(rows)[0]
            col_indices = np.where(cols)[0]
            
            rmin, rmax = row_indices[0], row_indices[-1]
            cmin, cmax = col_indices[0], col_indices[-1]
            
            # Explorable area = bounding box size
            explorable_cells = (rmax - rmin + 1) * (cmax - cmin + 1)
            
            # Coverage = known cells / explorable area
            coverage = (known_count / explorable_cells * 100) if explorable_cells > 0 else 0.0
            
            return min(100.0, coverage)
            
        except Exception as e:
            self.get_logger().error(f'Error calculating accurate coverage: {e}')
            return 0.0
    
    def calculate_redundancy(self):
        """Calculate redundancy for multi-robot exploration."""
        if self.mode == 'single' or len(self.robot_ids) < 2:
            return 0.0
        
        # Get explored cells for each robot
        robot_cells = [self.robot_explored_cells[rid] for rid in self.robot_ids]
        
        # Handle case when no cells explored yet
        if not any(robot_cells):
            return 0.0
        
        # Calculate union
        valid_sets = [cells for cells in robot_cells if cells]
        if not valid_sets:
            return 0.0
            
        all_cells = set.union(*valid_sets)
        
        if len(all_cells) == 0:
            return 0.0
        
        # Redundancy calculation
        redundant_count = 0
        for cell in all_cells:
            explored_by = sum(1 for cells in robot_cells if cell in cells)
            if explored_by > 1:
                redundant_count += (explored_by - 1)
        
        total_explorations = sum(len(cells) for cells in robot_cells)
        redundancy = (redundant_count / total_explorations * 100) if total_explorations > 0 else 0.0
        
        return redundancy
    
    def calculate_individual_contributions(self):
        """Calculate unique contribution of each robot."""
        if self.mode == 'single':
            return {}
        
        # Get all explored cells
        explored_sets = [self.robot_explored_cells[rid] for rid in self.robot_ids if self.robot_explored_cells[rid]]
        
        # Handle case when no cells explored yet
        if not explored_sets:
            return {}
        
        all_cells = set.union(*explored_sets)
        
        contributions = {}
        for robot_id in self.robot_ids:
            robot_cells = self.robot_explored_cells[robot_id]
            other_sets = [self.robot_explored_cells[other] 
                         for other in self.robot_ids 
                         if other != robot_id and self.robot_explored_cells[other]]
            
            other_cells = set.union(*other_sets) if other_sets else set()
            
            unique_cells = robot_cells - other_cells if other_cells else robot_cells
            total_cells = len(all_cells) if all_cells else 1
            
            contributions[robot_id] = {
                'total_explored': len(robot_cells),
                'unique_contribution': len(unique_cells),
                'unique_percentage': len(unique_cells) / total_cells * 100 if total_cells > 0 else 0
            }
        
        return contributions
    
    def log_coverage_timeseries(self):
        """Log coverage at 1-second intervals to separate file."""
        elapsed = time.time() - self.start_time
        
        # Get coverage using accurate method
        if self.mode == 'single':
            coverage = self.calculate_coverage_accurate(self.robot_maps[''])
        else:
            # Multi-robot: use average of individual robot maps
            coverages = []
            for robot_id in self.robot_ids:
                if self.robot_maps[robot_id] is not None:
                    cov = self.calculate_coverage_accurate(self.robot_maps[robot_id])
                    coverages.append(cov)
            coverage = sum(coverages) / len(coverages) if coverages else 0.0
        
        # Simple timeseries entry
        entry = {
            'time': round(elapsed, 1),
            'coverage': round(coverage, 2)
        }
        
        self.coverage_timeseries.append(entry)
        
        # Auto-save coverage file every 10 seconds
        if len(self.coverage_timeseries) % 10 == 0:
            self.save_coverage_timeseries()
    
    def collect_metrics(self):
        """Collect comprehensive metrics at regular intervals."""
        elapsed = time.time() - self.start_time
        
        # Calculate coverage
        if self.mode == 'single':
            explored_cells = len(self.robot_explored_cells[''])
            coverage = self.calculate_coverage_accurate(self.robot_maps[''])
        else:
            # Multi-robot: get union of explored cells
            all_sets = [self.robot_explored_cells[rid] for rid in self.robot_ids if self.robot_explored_cells[rid]]
            explored_cells = len(set.union(*all_sets)) if all_sets else 0
            
            # Calculate average coverage from individual robot maps
            # (merged map has frame issues, so use individual maps)
            coverages = []
            for robot_id in self.robot_ids:
                if self.robot_maps[robot_id] is not None:
                    cov = self.calculate_coverage_accurate(self.robot_maps[robot_id])
                    coverages.append(cov)
            
            coverage = sum(coverages) / len(coverages) if coverages else 0.0
        
        # Calculate redundancy (multi-robot only)
        redundancy = self.calculate_redundancy()
        
        # Individual robot contributions
        contributions = self.calculate_individual_contributions()
        
        # Build data point
        point = {
            'time': round(elapsed, 1),
            'coverage': round(coverage, 2),
            'mode': self.mode
        }
        
        if self.mode != 'single':
            point['redundancy'] = round(redundancy, 2)
            point['robot_contributions'] = contributions
            
            # Add per-robot coverage
            for robot_id in self.robot_ids:
                robot_coverage = self.calculate_coverage_accurate(self.robot_maps[robot_id])
                point[f'{robot_id}_coverage'] = round(robot_coverage, 2)
        
        self.data_points.append(point)
        
        # Log progress with absolute cell count
        if len(self.data_points) % 10 == 0:
            log_msg = f'Time: {elapsed:.1f}s | Coverage: {coverage:.1f}% | Explored cells: {explored_cells}'
            if self.mode != 'single':
                log_msg += f' | Redundancy: {redundancy:.1f}%'
            self.get_logger().info(log_msg)
        
        # Auto-save periodically
        if len(self.data_points) % 30 == 0:
            self.save_data()
    
    def save_coverage_timeseries(self):
        """Save 1-second interval coverage data."""
        with open(self.coverage_file, 'w') as f:
            json.dump(self.coverage_timeseries, f, indent=2)
    
    def save_data(self):
        """Save comprehensive collected data to JSON file."""
        output_data = {
            'mode': self.mode,
            'start_time': self.start_time,
            'duration': time.time() - self.start_time,
            'robot_ids': self.robot_ids,
            'data_points': self.data_points
        }
        
        # Add summary statistics
        if self.data_points:
            coverages = [d['coverage'] for d in self.data_points]
            output_data['summary'] = {
                'final_coverage': coverages[-1],
                'max_coverage': max(coverages),
                'total_time': self.data_points[-1]['time'],
                'num_samples': len(self.data_points)
            }
            
            if self.mode != 'single':
                redundancies = [d['redundancy'] for d in self.data_points if 'redundancy' in d]
                if redundancies:
                    output_data['summary']['avg_redundancy'] = sum(redundancies) / len(redundancies)
                    output_data['summary']['final_redundancy'] = redundancies[-1]
        
        with open(self.output_file, 'w') as f:
            json.dump(output_data, f, indent=2)
        
        self.get_logger().info(f'Data saved to {self.output_file}')
    
    def shutdown(self):
        """Save data on shutdown."""
        self.get_logger().info('Shutting down, saving final data...')
        self.save_data()
        self.save_coverage_timeseries()
        
        # Print summary
        if self.data_points:
            print("\n" + "="*60)
            print(f"METRICS COLLECTION COMPLETE - {self.mode.upper()} MODE")
            print("="*60)
            print(f"Duration: {self.data_points[-1]['time']:.1f}s")
            print(f"Final Coverage: {self.data_points[-1]['coverage']:.1f}%")
            if self.mode != 'single':
                redundancies = [d['redundancy'] for d in self.data_points if 'redundancy' in d]
                if redundancies:
                    print(f"Final Redundancy: {redundancies[-1]:.1f}%")
                    print(f"Avg Redundancy: {sum(redundancies)/len(redundancies):.1f}%")
            print(f"\nData files:")
            print(f"  Comprehensive: {self.output_file}")
            print(f"  Coverage (1s):  {self.coverage_file}")
            print("="*60 + "\n")


def main():
    parser = argparse.ArgumentParser(description='Collect exploration metrics')
    parser.add_argument('--mode', 
                       choices=['single', 'frontier', 'hungarian'],
                       default='single',
                       help='Exploration mode to evaluate')
    
    args = parser.parse_args()
    
    rclpy.init()
    collector = ExplorationMetricsCollector(mode=args.mode)
    
    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        pass
    finally:
        collector.shutdown()
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
