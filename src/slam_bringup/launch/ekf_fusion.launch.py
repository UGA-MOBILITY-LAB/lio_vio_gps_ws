"""Launch robot_localization EKF + navsat_transform for multi-source fusion.

Fuses:
  - FAST-LIO2 odometry (/Odometry) — absolute pose
  - VINS-Fusion odometry (/vins_estimator/odometry) — differential
  - GPS (/gps/fix) via navsat_transform
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
    config_file = PathJoinSubstitution([
        slam_bringup_share, 'config', 'ekf_navsat.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'),

        # EKF filter node — fuses FAST-LIO2 + VINS-Fusion odometry
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ],
        ),

        # NavSat transform — converts GPS (lat/lon) to odometry frame
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('gps/fix', '/gps/fix'),
                ('imu', '/imu/data'),
                ('odometry/filtered', '/odometry/filtered'),
            ],
        ),
    ])
