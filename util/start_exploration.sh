#!/bin/bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
sleep 45
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True &
sleep 10
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True &
sleep 10
cd ~/ros2_ws && source install/setup.bash
ros2 launch explore_lite explore.launch.py &
sleep 5
python3 ~/ros2_ws/collect_metrics.py
