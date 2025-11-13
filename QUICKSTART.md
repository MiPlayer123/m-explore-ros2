# m-explore-ros2 Quick Start

ROS2 port of multi-robot autonomous exploration package for frontier-based exploration and multi-robot map merging. Tested on ROS2 Humble.

## Installation (macOS with Apple Silicon - Recommended)

**Note:** Gazebo Classic has stability issues on macOS Apple Silicon. Using DevContainer is recommended.

### Prerequisites
1. Install [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop)
2. Install [VS Code](https://code.visualstudio.com/)
3. Install the [Dev Containers extension](vscode:extension/ms-vscode-remote.remote-containers) in VS Code
4. Install [XQuartz](https://www.xquartz.org/) for GUI support

### Setup
```bash
# Clone the repository
cd <path_to_workspace>
git clone <repo_url>
cd m-explore-ros2

# Rename macOS DevContainer config
mv .devcontainer/devcontainer.macos.json .devcontainer/devcontainer.json
mv .devcontainer/docker-compose.macos.yml .devcontainer/docker-compose.yml

# Open in VS Code
code .
```

In VS Code:
1. Press `Cmd+Shift+P` and select "Dev Containers: Reopen in Container"
2. Wait for the container to build (first time takes 10-15 minutes)
3. Once inside the container, open a terminal and run:
```bash
cd /workspace/ros_ws
colcon build --symlink-install
source install/setup.bash
```

### Running in DevContainer
See "Running Exploration Demo" section below, but run all commands inside the VS Code integrated terminal (which is inside the container).

## Installation (Native Linux or conda)

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
