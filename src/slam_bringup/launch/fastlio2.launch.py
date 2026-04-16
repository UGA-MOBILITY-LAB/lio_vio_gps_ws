"""Launch FAST-LIO2 with configurable YAML and static TF body → base_link."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    slam_bringup_share = get_package_share_directory('slam_bringup')
    fast_lio_share = get_package_share_directory('fast_lio')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'config_file', default_value='fastlio2_carla.yaml',
            description='FAST-LIO2 config YAML filename (in slam_bringup/config/)'),

        # FAST-LIO2 mapping node
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    slam_bringup_share, 'config', config_file]),
                {'use_sim_time': use_sim_time},
            ],
        ),

        # NOTE: base_link's parent is now owned by the local EKF in
        # ekf_fusion.launch.py (ekf_filter_node_odom publishes
        # odom → base_link). FAST-LIO's body frame lives in an isolated
        # subtree (odom → camera_init → body via ekf_fusion's static and
        # FAST-LIO's own TF) — they don't need to share base_link.
    ])
