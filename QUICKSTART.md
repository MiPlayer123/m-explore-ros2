# m-explore-ros2 Quick Start

ROS2 port of multi-robot autonomous exploration package for frontier-based exploration and multi-robot map merging. Tested on ROS2 Humble.

## Installation

```bash
# Navigate to workspace
cd <path_to_m-explore-ros2>

# Activate ROS2 environment
conda activate ros_env

# Initialize and update rosdep (first time only)
rosdep init
rosdep update

# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Install Nav2 and TurtleBot3 packages
conda install -c robostack-staging -c conda-forge -y ros-humble-nav2-bringup ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo ros-humble-slam-toolbox

# Build workspace
colcon build --symlink-install

# Source workspace (use setup.bash if using bash)
source install/setup.zsh
```

## Running Exploration Demo

**Terminal 1 - Launch Nav2 with SLAM:**
```bash
cd <path_to_m-explore-ros2>
conda activate ros_env
source install/setup.zsh
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
```

**Terminal 2 - Launch Exploration:**
```bash
cd <path_to_m-explore-ros2>
conda activate ros_env
source install/setup.zsh
ros2 launch explore_lite explore.launch.py
```

**Optional - Visualize in RViz2:**
```bash
rviz2
# Add marker topic: explore/frontiers
```
