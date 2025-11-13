# Build and Test Instructions - Milestone 2

## Prerequisites

### 1. Install scipy (Required for Hungarian Algorithm)
```bash
pip install scipy numpy
```

### 2. Verify ROS2 Environment
```bash
echo $ROS_DISTRO
# Should output: humble (or your ROS2 distro)
```

---

## Build Instructions

### Option A: Build All Packages
```bash
cd /workspace/ros_ws  # Or your workspace directory
colcon build --symlink-install
source install/setup.bash  # or setup.zsh if using zsh
```

### Option B: Build Step-by-Step (Recommended for First Build)

```bash
cd /workspace/ros_ws

# Step 1: Build custom messages first
colcon build --packages-select explore_msgs
source install/setup.bash

# Step 2: Build coordinator (depends on messages)
colcon build --packages-select explore_coordinator
source install/setup.bash

# Step 3: Build explore_lite (depends on messages)
colcon build --packages-select explore_lite
source install/setup.bash

# Step 4: Build map_merge (optional, already built)
colcon build --packages-select multirobot_map_merge
source install/setup.bash
```

### Expected Output
- **No errors**
- explore_msgs should generate Python and C++ interfaces
- explore_coordinator should install Python node
- explore_lite should compile with new IG code

---

## Testing

### Test 1: Single Robot with Information Gain

**Terminal 1 - Nav2 + SLAM:**
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
```

**Terminal 2 - Exploration:**
```bash
source install/setup.bash
ros2 launch explore_lite explore.launch.py
```

**Terminal 3 - Monitor Frontiers:**
```bash
ros2 topic echo /explore/frontiers_array
```

**Expected:**
- Robot explores autonomously
- Frontiers published with IG values
- Better exploration behavior than baseline

---

### Test 2: Coordinator Only (No Robots)

**Terminal 1 - Launch Coordinator:**
```bash
source install/setup.bash
ros2 launch explore_coordinator coordinator.launch.py
```

**Expected:**
- Node starts successfully
- Waits for frontier data
- No errors about missing scipy

---

### Test 3: Multi-Robot Coordinated Exploration (Full System)

**Single Terminal Launch:**
```bash
source install/setup.bash
ros2 launch explore_coordinator coordinated_exploration.launch.py
```

This launches:
1. Gazebo with 2 TurtleBot3 robots
2. SLAM (gmapping) for each robot
3. Nav2 navigation for each robot
4. Exploration nodes for robot1 and robot2
5. Map merge node
6. Exploration coordinator

**Monitor in Separate Terminals:**

```bash
# Terminal 2: Watch coordinator assignments
ros2 topic echo /exploration_coordinator/assignments

# Terminal 3: Watch robot1 frontiers
ros2 topic echo /robot1/explore/frontiers_array

# Terminal 4: Watch robot2 frontiers
ros2 topic echo /robot2/explore/frontiers_array

# Terminal 5: RViz
rviz2
```

**Expected Behavior:**
- 2 robots spawn in Gazebo
- Both robots start exploring after ~50s init delay
- Coordinator assigns non-overlapping frontiers
- Robots explore different areas (reduced redundancy)
- Map merge shows combined map

---

## Debugging

### Check if scipy is installed:
```bash
python3 -c "import scipy; print(scipy.__version__)"
```

### Check messages are built:
```bash
ros2 interface list | grep explore_msgs
# Should show:
#   explore_msgs/msg/Frontier
#   explore_msgs/msg/FrontierArray
```

### Check coordinator parameters:
```bash
ros2 param list /exploration_coordinator
ros2 param get /exploration_coordinator distance_weight
ros2 param get /exploration_coordinator information_gain_weight
```

### Monitor coordinator logs:
```bash
ros2 run explore_coordinator coordinator --ros-args --log-level debug
```

### Verify frontier publishing:
```bash
ros2 topic hz /robot1/explore/frontiers_array
# Should show ~0.5 Hz (planner_frequency)
```

---

## Common Issues

### Issue 1: "scipy not found"
```bash
pip install scipy
# Or in conda:
conda install scipy
```

### Issue 2: "explore_msgs not found"
```bash
# Build messages first and source
colcon build --packages-select explore_msgs
source install/setup.bash
```

### Issue 3: "No action server available"
- Wait 60s after launching Nav2 for initialization
- Check if Nav2 is running: `ros2 topic list | grep navigate`

### Issue 4: "No frontiers available"
- Check SLAM is running: `ros2 topic echo /map --once`
- Verify explore node is running: `ros2 node list | grep explore`
- Increase `min_frontier_size` parameter if too restrictive

---

## Performance Metrics

### Collect Metrics During Test 3:
**Terminal:**
```bash
cd /workspace/ros_ws/src/m-explore-ros2
python3 util/collect_metrics.py
```

Let run for 3-5 minutes, then stop and analyze:
```bash
python3 util/analyze_baseline.py
```

### Expected Improvements vs Baseline:
- **Redundancy**: <24% (vs 30% baseline)
- **Time to 90%**: Competitive or faster
- **Coverage**: ≥74% plateau

---

## Next Steps After Successful Build

1. **Run baseline comparison:**
   - Test uncoordinated (nearest-frontier)
   - Test coordinated (Hungarian)
   - Compare redundancy metrics

2. **Parameter tuning:**
   - Adjust `information_gain_scale` (try 3.0, 5.0, 7.0)
   - Tune `distance_weight` (0.5, 1.0, 2.0)
   - Test `coordination_frequency` (1Hz, 2Hz, 4Hz)

3. **Multi-environment testing:**
   - TurtleBot3 World
   - AWS Small House
   - Clearpath Construction

4. **Statistical validation:**
   - 10+ runs per configuration
   - t-test for significance
   - Cohen's d for effect size

---

## Quick Reference Commands

```bash
# Build everything
colcon build --symlink-install && source install/setup.bash

# Test single robot IG
ros2 launch explore_lite explore.launch.py

# Test coordinated multi-robot
ros2 launch explore_coordinator coordinated_exploration.launch.py

# Monitor coordinator
ros2 topic echo /exploration_coordinator/assignments

# Check IG values
ros2 topic echo /robot1/explore/frontiers_array --field frontiers[0].information_gain

# Stop all
Ctrl+C in all terminals
```

---

## Success Criteria Checklist

- [ ] All packages build without errors
- [ ] scipy import works in coordinator node
- [ ] Single robot publishes frontiers with IG values
- [ ] Coordinator node starts and discovers robots
- [ ] Multi-robot launch brings up 2 robots
- [ ] Coordinator assigns frontiers to both robots
- [ ] Robots explore non-overlapping areas
- [ ] Redundancy < 24% measured

If all checks pass, implementation is complete and ready for evaluation!
