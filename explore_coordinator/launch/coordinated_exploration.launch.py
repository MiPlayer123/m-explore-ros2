"""
Launch file for coordinated multi-robot exploration.

Starts:
1. Multi-robot Gazebo simulation
2. SLAM for each robot
3. Nav2 navigation for each robot
4. Exploration nodes for each robot
5. Map merge node
6. Exploration coordinator node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Directories
    map_merge_dir = get_package_share_directory('multirobot_map_merge')
    explore_dir = get_package_share_directory('explore_lite')
    coordinator_dir = get_package_share_directory('explore_coordinator')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # 1. Launch multi-robot simulation
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                map_merge_dir,
                'launch',
                'tb3_simulation',
                'multi_tb3_simulation_launch.py'
            )
        ),
        launch_arguments={
            'slam_gmapping': 'True',
            'known_init_poses': 'True',
            'use_sim_time': use_sim_time
        }.items()
    )

    # 2. Launch map merge
    map_merge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(map_merge_dir, 'launch', 'map_merge.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Launch exploration for robot1
    explore_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(explore_dir, 'launch', 'explore.launch.py')
        ),
        launch_arguments={
            'namespace': 'robot1',
            'use_sim_time': use_sim_time
        }.items()
    )

    # 4. Launch exploration for robot2
    explore_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(explore_dir, 'launch', 'explore.launch.py')
        ),
        launch_arguments={
            'namespace': 'robot2',
            'use_sim_time': use_sim_time
        }.items()
    )

    # 5. Launch coordinator
    coordinator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coordinator_dir, 'launch', 'coordinator.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Add all actions
    ld.add_action(declare_use_sim_time)
    ld.add_action(simulation_launch)
    ld.add_action(map_merge_launch)
    ld.add_action(explore_robot1)
    ld.add_action(explore_robot2)
    ld.add_action(coordinator_launch)

    return ld
