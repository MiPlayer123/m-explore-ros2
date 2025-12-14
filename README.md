# Multi-Robot Frontier Exploration with Information-Theoretic Task Allocation

An enhanced ROS2 multi-robot exploration system that combines **information-theoretic frontier scoring** with **optimal task allocation** via the Hungarian algorithm. Built on top of [m-explore](https://github.com/hrnr/m-explore), this project achieves **4.8x speedup** over single-robot exploration through intelligent coordination.

## Key Features

- **Information Gain Scoring**: Raycasting-based metric that estimates expected map expansion from each frontier, prioritizing high-value exploration targets over simple nearest-frontier heuristics
- **Hungarian Algorithm Coordination**: Optimal robot-frontier assignment that minimizes redundant coverage and maximizes collective exploration efficiency
- **Centralized Coordinator**: A 2 Hz coordinator node that subscribes to frontier data from all robots and dispatches globally optimal navigation goals
- **ROS2/Nav2 Integration**: Seamless deployment on TurtleBot3 platforms with custom messages for frontier geometry and information gain

## Results

| Configuration | Time to 90% Coverage | Speedup |
|--------------|---------------------|---------|
| Single Robot | 142.8s | 1.0x |
| Two Robot Coordinated (Ours) | 29.8s | **4.8x** |

Tested in TurtleBot3 World (~100m²) and warehouse environments (~112m²). See `latex_report/final_report.pdf` for full experimental details.

## Architecture

```
Robot 1: SLAM → Costmap → Frontier Detection (with IG scoring)
                ↓
        /robot1/frontiers_array
                ↓
    ┌─────────────────────────────────┐
    │  Exploration Coordinator (2 Hz) │
    │  Cost Matrix → Hungarian Solver │
    └─────────────────────────────────┘
                ↓
        /robot1/navigate_to_pose
```

Each robot independently runs SLAM and frontier detection, publishing scored frontiers. The coordinator optimally assigns robots to frontiers and dispatches Nav2 goals.

---

## Contents
1. [Installation](#installation)
2. [Single Robot Exploration](#single-robot-exploration)
3. [Coordinated Multi-Robot Exploration](#coordinated-multi-robot-exploration)
4. [Multi-Robot Map Merge](#multirobot-map-merge)
5. [Packages](#packages)

## Installation

### Dependencies

```bash
# Install Nav2 and TurtleBot3 simulation
sudo apt install ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-turtlebot3-gazebo

# Install scipy for Hungarian algorithm
pip install scipy numpy
```

### Building

```bash
cd <your_ros2_ws>/src
git clone https://github.com/MiPlayer123/m-explore-ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## Single Robot Exploration

### Simulation with a TB3 robot
https://user-images.githubusercontent.com/8033598/128805356-be90a880-16c6-4fc9-8f54-e3302873dc8c.mp4

### Running the Demo

**Terminal 1** - Launch Nav2 with SLAM:
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
```

**Terminal 2** - Launch exploration:
```bash
ros2 launch explore_lite explore.launch.py
```

### Additional Features

**Stop/Resume exploration:**
```bash
ros2 topic pub /explore/resume std_msgs/msg/Bool "data: false"  # Stop
ros2 topic pub /explore/resume std_msgs/msg/Bool "data: true"   # Resume
```

**Return to initial pose:** Set `return_to_init: True` in params.

---

## Coordinated Multi-Robot Exploration

Our enhanced coordinated exploration system with information-theoretic scoring.

### Running the Demo

**Terminal 1** - Launch multi-robot simulation:
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
ros2 launch multirobot_map_merge multi_tb3_simulation_launch.py slam_gmapping:=True
```

**Terminal 2** - Launch map merge:
```bash
ros2 launch multirobot_map_merge map_merge.launch.py
```

**Terminal 3** - Launch coordinator:
```bash
ros2 launch explore_coordinator coordinated_exploration.launch.py
```

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `planner_frequency` | 0.5 Hz | Frontier replanning rate |
| `information_gain_scale` | 5.0 | Weight for IG in cost function |
| `min_frontier_size` | 0.5m | Minimum frontier size to consider |

---

## Multirobot Map Merge

Supports both known and unknown initial poses for map merging.

### Known initial poses

https://user-images.githubusercontent.com/8033598/144522712-c31fb4bb-bb5a-4859-b3e1-8ad665f80696.mp4

### Unknown initial poses
Works best if robots start close together (< 3 meters).

https://user-images.githubusercontent.com/8033598/144522696-517d54fd-74d0-4c55-9aca-f1b9679afb3e.mp4

### SLAM Requirements

For map merging, use [slam_gmapping](https://github.com/charlielito/slam_gmapping/tree/feature/namespace_launch) (with namespace support):

```bash
cd <your_ros2_ws>/src
git clone https://github.com/charlielito/slam_gmapping.git --branch feature/namespace_launch
cd ..
colcon build --symlink-install --packages-up-to slam_gmapping
```

**Note**: [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) support available on [experimental branch](https://github.com/robo-friends/m-explore-ros2/tree/feature/slam_toolbox_compat).

---

## Packages

| Package | Description |
|---------|-------------|
| `explore_lite` | Frontier-based exploration with information gain scoring |
| `explore_coordinator` | Centralized Hungarian algorithm coordinator |
| `explore_msgs` | Custom messages for frontier data with IG |
| `multirobot_map_merge` | Map merging for multi-robot systems |

---

## Simulation

A standalone Python simulation is available in `simulation/` for testing the coordination algorithm without ROS:

```bash
cd simulation
python3 warehouse_final.py
```

---

## Citation

If you use this work, please cite:

```
Multi-Robot Frontier Exploration with Information-Theoretic Task Allocation
Mikul Saravanan, Samay Lakhani, Jeffrey Kravitz
```

## License

Packages are licensed under BSD license. See respective files for details.

## Acknowledgments

- Original [m-explore](https://github.com/hrnr/m-explore) by Jiří Hörner
- [Nav2](https://navigation.ros.org/) team for the navigation stack
- ROBOTIS for TurtleBot3 simulation environment
