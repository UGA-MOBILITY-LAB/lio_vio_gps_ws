"""Full CARLA simulation pipeline: bridge + FAST-LIO2 + VINS-Fusion + EKF + RViz.

Launches:
  1. carla_live_publisher — CARLA sensor bridge + /clock server
  2. FAST-LIO2 — LiDAR-Inertial odometry
  3. VINS-Fusion — Visual-Inertial odometry (mono)
  4. robot_localization EKF + navsat_transform — multi-source fusion
  5. Static TF: body → base_link
  6. RViz2 with unified slam_carla.rviz layout (toggle with rviz:=false)

All SLAM nodes use use_sim_time:=true.  The CARLA bridge does NOT
(it is the clock source).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node  # noqa: F401  # used below


def generate_launch_description():
    slam_bringup_share = get_package_share_directory('slam_bringup')
    launch_dir = os.path.join(slam_bringup_share, 'launch')
    rviz_config = os.path.join(slam_bringup_share, 'rviz', 'slam_carla.rviz')

    carla_host = LaunchConfiguration('carla_host')
    carla_port = LaunchConfiguration('carla_port')
    town = LaunchConfiguration('town')
    rviz = LaunchConfiguration('rviz')
    vins = LaunchConfiguration('vins')

    return LaunchDescription([
        # ── Launch arguments ──────────────────────────────────────
        DeclareLaunchArgument('carla_host', default_value='localhost'),
        DeclareLaunchArgument('carla_port', default_value='2000'),
        DeclareLaunchArgument('town', default_value='Town10HD'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 with unified slam_carla layout',
        ),
        DeclareLaunchArgument(
            'vins',
            default_value='false',
            description='Launch VINS-Fusion (requires a VINS-format config; off until vins_carla_mono.yaml is converted)',
        ),

        # ── 1. CARLA bridge (clock server — NOT use_sim_time) ─────
        Node(
            package='slam_bringup',
            executable='carla_bridge',
            name='carla_bridge',
            output='screen',
            parameters=[{
                'carla_host': carla_host,
                'carla_port': carla_port,
                'town': town,
                'use_sim_time': False,
            }],
        ),

        # ── 2. FAST-LIO2 ─────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'fastlio2.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'config_file': 'fastlio2_carla.yaml',
            }.items(),
        ),

        # ── 3. VINS-Fusion (off by default) ──────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'vins_fusion.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'config_file': 'vins_carla_mono.yaml',
            }.items(),
            condition=IfCondition(vins),
        ),

        # ── 4. EKF fusion + NavSat ───────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'ekf_fusion.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
            }.items(),
        ),

        # ── 5. RViz2 (unified mapping + localization view) ───────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz),
        ),
    ])
