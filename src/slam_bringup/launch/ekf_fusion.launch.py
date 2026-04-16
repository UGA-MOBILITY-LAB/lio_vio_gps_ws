"""Dual-EKF + NavSat launch for map-frame GPS anchoring.

Mirrors robot_localization's `dual_ekf_navsat_example.launch.py`:

    ekf_filter_node_odom (local)   →  odom  → base_link  (→ /odometry/local)
    ekf_filter_node_map  (global)  →  map   → odom       (→ /odometry/global)
    navsat_transform_node          →  /gps/fix + /odometry/global → /odometry/gps

FAST-LIO2 publishes /Odometry in `camera_init`.  A static identity TF
`odom → camera_init` lets the EKFs resolve that frame to their world
frames.  The global EKF consumes LIO in *differential* mode (LIO origin
is the spawn point, not the GPS datum — feeding its absolute pose would
fight GPS) plus GPS in absolute mode to own the true global pose.
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

        # Static TF: odom → camera_init (identity).
        # Lets robot_localization resolve FAST-LIO's camera_init-framed
        # /Odometry into either EKF's world frame.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_camera_init',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ── Local EKF (odom frame) ─────────────────────────────────
        # Fuses LIO (+ VIO), publishes odom → base_link and /odometry/local.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/local')],
        ),

        # ── Global EKF (map frame, with GPS) ───────────────────────
        # Fuses LIO differentially + absolute /odometry/gps, publishes
        # map → odom (the drift-correction transform) and /odometry/global.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/global')],
        ),

        # ── NavSat transform: /gps/fix + global pose → /odometry/gps
        # navsat subscribes to the GLOBAL EKF's output (example-conformant)
        # so its heading / offset reasoning stays in the map frame.
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu/data', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('gps/filtered', '/gps/filtered'),
                ('odometry/gps', '/odometry/gps'),
                ('odometry/filtered', '/odometry/global'),
            ],
        ),
    ])
