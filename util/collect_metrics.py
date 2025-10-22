#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import json
import time

class SimpleMetrics(Node):
    def __init__(self):
        super().__init__('simple_metrics')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.start_time = time.time()
        self.data_points = []
        
    def map_callback(self, msg):
        data = np.array(msg.data)
        total = len(data)
        known = np.sum(data != -1)
        coverage = (known / total * 100) if total > 0 else 0
        
        elapsed = time.time() - self.start_time
        
        point = {
            'time': round(elapsed, 1),
            'coverage': round(coverage, 1)
        }
        
        self.data_points.append(point)
        self.get_logger().info(f'Time: {elapsed:.1f}s | Coverage: {coverage:.1f}%')
        
        # Save every update
        with open('/tmp/metrics.json', 'w') as f:
            json.dump(self.data_points, f, indent=2)

def main():
    rclpy.init()
    node = SimpleMetrics()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
