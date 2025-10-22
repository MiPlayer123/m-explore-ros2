# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 port of the m-explore package for multi-robot autonomous exploration. The repository contains two main ROS2 packages:

1. **explore_lite** - Frontier-based autonomous exploration for single robots
2. **multirobot_map_merge** - Map merging for multi-robot systems with known/unknown initial poses

Tested on ROS2 Eloquent, Dashing, Foxy, Galactic, and Humble distributions.

## Environment Setup

This project uses a conda environment for ROS2 development:

```bash
# Activate ROS2 environment
conda activate ros_env

# Source workspace after building (use setup.bash if using bash)
source install/setup.zsh
```

## Build Commands

```bash
# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build entire workspace
colcon build --symlink-install

# Build specific package
colcon build --symlink-install --packages-select explore_lite
colcon build --symlink-install --packages-select multirobot_map_merge

# Build with dependencies
colcon build --symlink-install --packages-up-to explore_lite
```

## Running Exploration

### Single Robot Exploration

Two terminals required:

**Terminal 1 - Nav2 stack with SLAM:**
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
```

**Terminal 2 - Exploration node:**
```bash
ros2 launch explore_lite explore.launch.py
```

**Run with custom parameters:**
```bash
ros2 run explore_lite explore --ros-args --params-file <path_to_ws>/explore/config/params.yaml
```

**Stop/Resume exploration:**
```bash
# Pause exploration
ros2 topic pub /explore/resume std_msgs/msg/Bool "data: false"

# Resume exploration
ros2 topic pub /explore/resume std_msgs/msg/Bool "data: true"
```

### Multi-Robot Map Merge

**Terminal 1 - Launch multi-robot simulation:**
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models

# With known initial poses (default)
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_gmapping:=True

# With unknown initial poses
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_gmapping:=True known_init_poses:=False
```

**Terminal 2 - Map merge node:**
```bash
ros2 launch multirobot_map_merge map_merge.launch.py
```

**Visualize merged map:**
```bash
rviz2 -d <path_to_ws>/src/m-explore-ros2/map_merge/launch/map_merge.rviz
```

## Automated Exploration Script

The `util/start_exploration.sh` script automates the entire exploration workflow:
- Launches TurtleBot3 Gazebo simulation
- Starts SLAM (cartographer)
- Launches Nav2 navigation stack
- Runs exploration node
- Collects metrics automatically

## Metrics Collection

**Collect coverage metrics during exploration:**
```bash
python3 util/collect_metrics.py
```
- Subscribes to `/map` topic
- Tracks coverage percentage over time
- Saves data to `/tmp/metrics.json`

**Plot metrics:**
```bash
python3 util/plot_metrics.py
```
- Reads `/tmp/metrics.json`
- Generates coverage vs. time plot
- Saves to `/tmp/exploration_plot.png`

## Architecture

### explore_lite Package

**Core components:**
- `explore.cpp` - Main exploration node using frontier-based exploration
- `frontier_search.cpp` - Identifies unexplored frontiers in the costmap
- `costmap_client.cpp` - Interfaces with Nav2 costmap

**Key design:**
- Uses Nav2 action client (`NavigateToPose`) to send goal poses
- Frontier scoring based on potential_scale, gain_scale, and orientation_scale
- Integrates with Nav2's costmap_2d for obstacle information
- Publishes frontier visualization markers to `explore/frontiers`

**Important parameters (config/params.yaml):**
- `robot_base_frame`: Robot's base frame (default: base_link)
- `return_to_init`: Return to starting position after exploration
- `planner_frequency`: How often to recompute frontiers (Hz)
- `min_frontier_size`: Minimum frontier size to consider (meters)
- `progress_timeout`: Timeout if no progress made (seconds)

### multirobot_map_merge Package

**Core components:**
- `map_merge.cpp` - Main node that discovers robot maps and merges them
- `combine_grids/` - Grid merging algorithms

**Key design:**
- Dynamically discovers robot namespaces and subscribes to their maps
- Two modes: known initial poses (uses TF tree) or unknown (uses feature matching)
- Uses OpenCV for map alignment when initial poses unknown
- Publishes merged map on configurable topic (default: `/map`)

**Important parameters:**
- `known_init_poses`: Whether robot starting positions are known
- `merging_rate`: Frequency of map merging (Hz)
- `robot_map_topic`: Topic name for individual robot maps
- `world_frame`: Global coordinate frame for merged map
- `estimation_confidence`: Confidence threshold for pose estimation

## Dependencies

**Critical ROS2 packages:**
- `nav2-bringup` - Navigation stack
- `turtlebot3` and `turtlebot3-gazebo` - Simulation
- `slam-toolbox` or `slam_gmapping` - SLAM (gmapping requires special fork for namespacing)

**For slam_gmapping support:**
```bash
cd <ros2_ws/src>
git clone https://github.com/charlielito/slam_gmapping.git --branch feature/namespace_launch
colcon build --symlink-install --packages-up-to slam_gmapping
```

## SLAM Backend Notes

- Default demos use `slam_gmapping` (ROS1 port, requires fork for multi-robot)
- `slam_toolbox` support available on experimental branch: `feature/slam_toolbox_compat`
- Map merge logic currently optimized for gmapping-style maps
- Known initial poses mode gives best merging results

## Topic Structure

**explore_lite:**
- Subscribes: `/map`, `/map_updates` (costmap data)
- Publishes: `explore/frontiers` (visualization_msgs::MarkerArray)
- Actions: Uses `navigate_to_pose` action from Nav2
- Services: `explore/resume` (std_msgs::Bool)

**multirobot_map_merge:**
- Subscribes: `/<robot_ns>/map` for each discovered robot
- Publishes: `/map` (merged occupancy grid)
- Uses TF tree for robot pose transforms (known poses mode)

## Data Directory

Contains logged exploration metrics from test runs:
- `metrics.json` - Manual exploration run data
- `metrics_auto.json` - Automated exploration run data

These are used for analyzing exploration performance and coverage efficiency.
