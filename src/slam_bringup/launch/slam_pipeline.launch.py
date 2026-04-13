"""Real vehicle SLAM pipeline: FAST-LIO2 + VINS-Fusion + EKF (no CARLA).

Same as carla_full.launch.py but without the CARLA bridge and with
use_sim_time:=false.  Expects real sensor drivers to publish:
  /imu/data, /lidar/points, /camera/image_color, /gps/fix
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_bringup_share = get_package_share_directory('slam_bringup')
    launch_dir = os.path.join(slam_bringup_share, 'launch')

    fastlio2_config = LaunchConfiguration('fastlio2_config')
    vins_config = LaunchConfiguration('vins_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fastlio2_config', default_value='fastlio2_real.yaml',
            description='FAST-LIO2 config for real vehicle'),
        DeclareLaunchArgument(
            'vins_config', default_value='vins_real_mono.yaml',
            description='VINS-Fusion config for real vehicle'),

        # ── FAST-LIO2 ────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'fastlio2.launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'config_file': fastlio2_config,
            }.items(),
        ),

        # ── VINS-Fusion ──────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'vins_fusion.launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'config_file': vins_config,
            }.items(),
        ),

        # ── EKF fusion + NavSat ──────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'ekf_fusion.launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
            }.items(),
        ),

        # ── Static TF: base_link → sensor frames ─────────────────
        # For real vehicle, publish sensor TFs here (adjust offsets
        # to match your actual sensor mounting positions).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'lidar_link'],
            # TODO: Set real offsets, e.g. ['0', '0', '1.5', '0', '0', '0', ...]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
            # TODO: Set real offsets
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gps',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps_link'],
            # TODO: Set real offsets
        ),
    ])
