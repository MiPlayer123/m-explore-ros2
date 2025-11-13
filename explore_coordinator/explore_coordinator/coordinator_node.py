#!/usr/bin/env python3
"""
Multi-Robot Exploration Coordinator Node

Uses Hungarian algorithm (scipy.optimize.linear_sum_assignment) to optimally
assign frontiers to robots based on a cost matrix considering:
- Distance cost
- Information gain reward
- Assignment history (prevent oscillation)
- Cross-robot penalties

Architecture:
1. Subscribe to FrontierArray from each robot
2. Build global cost matrix for all robot-frontier pairs
3. Solve assignment problem with Hungarian algorithm
4. Send navigation goals to robots' Nav2 action servers
5. Track assignment history to prevent flip-flopping

Author: Milestone 2 Team
Date: November 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from explore_msgs.msg import FrontierArray, Frontier
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
import numpy as np
try:
    from scipy.optimize import linear_sum_assignment
except ImportError:
    print("ERROR: scipy not installed. Run: pip install scipy")
    raise
from collections import defaultdict, deque
import time


class ExplorationCoordinator(Node):
    """
    Central coordinator for multi-robot frontier exploration.

    Receives frontiers from all robots, computes optimal assignment using
    Hungarian algorithm, and dispatches navigation goals.
    """

    def __init__(self):
        super().__init__('exploration_coordinator')

        # Declare parameters
        self.declare_parameter('robot_names', ['robot1', 'robot2'])
        self.declare_parameter('coordination_frequency', 2.0)  # Hz
        self.declare_parameter('distance_weight', 1.0)
        self.declare_parameter('information_gain_weight', 5.0)
        self.declare_parameter('history_penalty', 0.5)
        self.declare_parameter('history_window', 5)  # Remember last 5 assignments
        self.declare_parameter('min_reassignment_time', 10.0)  # seconds
        self.declare_parameter('use_sim_time', True)

        # Get parameters
        self.robot_names = self.get_parameter('robot_names').value
        self.coord_freq = self.get_parameter('coordination_frequency').value
        self.distance_weight = self.get_parameter('distance_weight').value
        self.ig_weight = self.get_parameter('information_gain_weight').value
        self.history_penalty = self.get_parameter('history_penalty').value
        self.history_window = self.get_parameter('history_window').value
        self.min_reassignment_time = self.get_parameter('min_reassignment_time').value

        self.get_logger().info(f'Starting coordinator for robots: {self.robot_names}')
        self.get_logger().info(f'Weights - Distance: {self.distance_weight}, IG: {self.ig_weight}')

        # Data structures
        self.frontier_data = {}  # {robot_id: FrontierArray}
        self.current_assignments = {}  # {robot_id: frontier_id}
        self.assignment_history = defaultdict(lambda: deque(maxlen=self.history_window))
        self.last_assignment_time = {}  # {robot_id: timestamp}
        self.action_clients = {}  # {robot_id: ActionClient}

        # Create subscribers for each robot's frontiers
        self.frontier_subscribers = {}
        for robot_name in self.robot_names:
            topic = f'/{robot_name}/explore/frontiers_array'
            self.frontier_subscribers[robot_name] = self.create_subscription(
                FrontierArray,
                topic,
                lambda msg, name=robot_name: self.frontier_callback(msg, name),
                10
            )
            self.get_logger().info(f'Subscribed to {topic}')

            # Create action client for navigation
            action_topic = f'/{robot_name}/navigate_to_pose'
            self.action_clients[robot_name] = ActionClient(
                self,
                NavigateToPose,
                action_topic
            )
            self.last_assignment_time[robot_name] = 0.0

        # Create timer for coordination loop
        self.coordination_timer = self.create_timer(
            1.0 / self.coord_freq,
            self.coordination_callback
        )

        self.get_logger().info('Exploration Coordinator initialized')

    def frontier_callback(self, msg: FrontierArray, robot_id: str):
        """Store incoming frontier data from robots."""
        self.frontier_data[robot_id] = msg
        self.get_logger().debug(
            f'Received {len(msg.frontiers)} frontiers from {robot_id}'
        )

    def build_cost_matrix(self):
        """
        Build cost matrix for Hungarian algorithm.

        Returns:
            cost_matrix (np.array): NxM matrix where N=robots, M=frontiers
            robot_list (list): Ordered list of robot names
            frontier_list (list): Ordered list of (robot_id, frontier) tuples
        """
        robot_list = []
        frontier_list = []

        # Collect all robots and frontiers
        for robot_id in self.robot_names:
            if robot_id in self.frontier_data:
                frontiers = self.frontier_data[robot_id].frontiers
                if len(frontiers) > 0:
                    robot_list.append(robot_id)
                    for frontier in frontiers:
                        frontier_list.append((robot_id, frontier))

        if not robot_list or not frontier_list:
            return None, None, None

        n_robots = len(robot_list)
        n_frontiers = len(frontier_list)

        # Initialize cost matrix
        cost_matrix = np.full((n_robots, n_frontiers), np.inf)

        # Fill cost matrix
        for i, robot_id in enumerate(robot_list):
            for j, (source_robot, frontier) in enumerate(frontier_list):
                # Base cost: distance - information_gain
                distance_cost = self.distance_weight * frontier.min_distance
                ig_reward = self.ig_weight * frontier.information_gain
                base_cost = distance_cost - ig_reward

                # History penalty: discourage reassigning recently assigned frontiers
                history_cost = 0.0
                if frontier.id in self.assignment_history[robot_id]:
                    recent_count = list(self.assignment_history[robot_id]).count(frontier.id)
                    history_cost = self.history_penalty * recent_count

                # Cross-robot penalty: prefer robots to explore their own frontiers
                cross_robot_cost = 2.0 if robot_id != source_robot else 0.0

                cost_matrix[i, j] = base_cost + history_cost + cross_robot_cost

        return cost_matrix, robot_list, frontier_list

    def coordination_callback(self):
        """
        Main coordination loop.

        1. Build cost matrix from all frontier data
        2. Solve assignment problem with Hungarian algorithm
        3. Send navigation goals to robots
        4. Update assignment history
        """
        # Build cost matrix
        cost_matrix, robot_list, frontier_list = self.build_cost_matrix()

        if cost_matrix is None:
            self.get_logger().debug('No frontiers available for assignment', throttle_duration_sec=5.0)
            return

        # Pad matrix if needed (more frontiers than robots or vice versa)
        n_robots, n_frontiers = cost_matrix.shape
        max_dim = max(n_robots, n_frontiers)

        if n_robots != n_frontiers:
            # Hungarian algorithm requires square matrix
            padded = np.full((max_dim, max_dim), np.max(cost_matrix) * 100)
            padded[:n_robots, :n_frontiers] = cost_matrix
            cost_matrix = padded

        # Solve assignment problem
        try:
            row_indices, col_indices = linear_sum_assignment(cost_matrix)
        except Exception as e:
            self.get_logger().error(f'Hungarian algorithm failed: {e}')
            return

        # Process assignments
        current_time = self.get_clock().now().nanoseconds / 1e9
        for robot_idx, frontier_idx in zip(row_indices, col_indices):
            # Skip dummy assignments
            if robot_idx >= n_robots or frontier_idx >= n_frontiers:
                continue

            robot_id = robot_list[robot_idx]
            source_robot, frontier = frontier_list[frontier_idx]

            # Check if enough time has passed since last assignment
            if current_time - self.last_assignment_time[robot_id] < self.min_reassignment_time:
                if self.current_assignments.get(robot_id) is not None:
                    self.get_logger().debug(
                        f'{robot_id}: Skipping reassignment (too soon)',
                        throttle_duration_sec=5.0
                    )
                    continue

            # Check if this is a new assignment
            if self.current_assignments.get(robot_id) != frontier.id:
                self.send_navigation_goal(robot_id, frontier)
                self.current_assignments[robot_id] = frontier.id
                self.assignment_history[robot_id].append(frontier.id)
                self.last_assignment_time[robot_id] = current_time

                # Calculate original cost (before padding)
                orig_cost = cost_matrix[robot_idx, frontier_idx] if robot_idx < n_robots and frontier_idx < n_frontiers else np.inf

                self.get_logger().info(
                    f'Assigned frontier {frontier.id} to {robot_id} '
                    f'(cost: {orig_cost:.2f}, dist: {frontier.min_distance:.2f}m, IG: {frontier.information_gain:.2f})'
                )

    def send_navigation_goal(self, robot_id: str, frontier: Frontier):
        """
        Send navigation goal to robot's action server.

        Args:
            robot_id: Robot identifier
            frontier: Frontier to navigate to
        """
        if robot_id not in self.action_clients:
            self.get_logger().error(f'No action client for {robot_id}')
            return

        client = self.action_clients[robot_id]

        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f'Navigation action server for {robot_id} not available',
                throttle_duration_sec=10.0
            )
            return

        # Build navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position = frontier.centroid
        goal_msg.pose.pose.orientation.w = 1.0  # No specific orientation

        # Send goal asynchronously
        self.get_logger().info(
            f'Sending goal to {robot_id}: '
            f'({frontier.centroid.x:.2f}, {frontier.centroid.y:.2f})'
        )

        future = client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda f, rid=robot_id: self.goal_response_callback(f, rid)
        )

    def goal_response_callback(self, future, robot_id: str):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'Goal rejected by {robot_id}')
            # Remove from current assignments so we can reassign
            self.current_assignments.pop(robot_id, None)
        else:
            self.get_logger().debug(f'Goal accepted by {robot_id}')


def main(args=None):
    rclpy.init(args=args)
    coordinator = ExplorationCoordinator()

    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
