import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Get config file path
    config = os.path.join(
        get_package_share_directory('explore_coordinator'),
        'config',
        'coordinator_params.yaml'
    )

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Coordinator node
    coordinator_node = Node(
        package='explore_coordinator',
        executable='coordinator',
        name='exploration_coordinator',
        output='screen',
        parameters=[config, {'use_sim_time': use_sim_time}]
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(coordinator_node)

    return ld
