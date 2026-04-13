"""Launch VINS-Fusion mono+IMU estimator.

VINS-Fusion takes the config file path as argv[1], not as a ROS parameter.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    slam_bringup_share = get_package_share_directory('slam_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'config_file', default_value='vins_carla_mono.yaml',
            description='VINS-Fusion config YAML filename (in slam_bringup/config/)'),

        # VINS-Fusion estimator
        # Config path passed as argv[1] (not a ROS parameter)
        Node(
            package='vins',
            executable='vins_node',
            name='vins_estimator',
            output='screen',
            arguments=[
                PathJoinSubstitution([
                    slam_bringup_share, 'config', config_file]),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
