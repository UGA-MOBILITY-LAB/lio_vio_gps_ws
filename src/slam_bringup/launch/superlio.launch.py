"""Launch Super-LIO (octree voxel map, z-drift-resistant) with configurable YAML."""

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
            'config_file', default_value='superlio_carla.yaml',
            description='Super-LIO config YAML filename (in slam_bringup/config/)'),

        # Super-LIO main node.
        # Publishes /lio/odom (lidar rate), /lio/imu/odom, /lio/robo/odom,
        # /lio/path, /lio/cloud_world — all in frame_id "world".
        # ekf_fusion.launch.py publishes a static TF odom → world (identity)
        # so robot_localization resolves these into the odom/map frames.
        Node(
            package='super_lio',
            executable='super_lio_node',
            name='super_lio_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    slam_bringup_share, 'config', config_file]),
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
