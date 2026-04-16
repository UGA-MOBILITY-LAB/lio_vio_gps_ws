"""Dual-EKF + NavSat launch for map-frame GPS anchoring.

Topology:
    ekf_filter_node_odom (local)   →  odom  → base_link
    ekf_filter_node_map  (global)  →  map   → odom
    navsat_transform_node          →  /gps/fix + /odometry/filtered → /odometry/gps

The local EKF fuses LIO (+ optional VIO) and owns odom → base_link.
The global EKF additionally fuses GPS and owns map → odom, correcting the
odom frame's accumulated drift against the GPS datum.

FAST-LIO2's /Odometry has frame_id=camera_init.  A static identity TF
odom → camera_init lets robot_localization resolve the measurement's
frame to each EKF's world frame.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_bringup_share = get_package_share_directory('slam_bringup')
    ekf_config = os.path.join(
        slam_bringup_share, 'config', 'ekf_navsat.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'),

        # Static TF: odom → camera_init (identity)
        # FAST-LIO2 publishes /Odometry in camera_init.  This edge lets the
        # two EKFs transform the measurement into their world frames
        # (odom for the local EKF, map for the global one via map→odom).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_camera_init',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ── Local EKF (odom frame) ─────────────────────────────────
        # Fuses LIO + VIO, publishes odom → base_link.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),

        # ── Global EKF (map frame, with GPS) ───────────────────────
        # Fuses LIO + GPS, publishes map → odom.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/filtered_map')],
        ),

        # ── NavSat transform: /gps/fix + local pose → /odometry/gps ─
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', '/odometry/filtered'),
                ('odometry/gps', '/odometry/gps'),
            ],
        ),
    ])
