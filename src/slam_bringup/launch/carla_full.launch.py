"""Full CARLA simulation pipeline: bridge + Super-LIO + FAST-LIO2 + VINS-Fusion + EKF + RViz.

Launches:
  1. carla_live_publisher — CARLA sensor bridge + /clock server
  2. Super-LIO — LiDAR-Inertial odometry with octree voxel map (feeds EKF)
  3. FAST-LIO2 — LiDAR-Inertial odometry, kept on for side-by-side comparison
  4. VINS-Fusion — Visual-Inertial odometry (mono), off by default
  5. robot_localization EKF + navsat_transform — multi-source fusion
  6. RViz2 with unified slam_carla.rviz layout (toggle with rviz:=false)

The global/local EKFs consume Super-LIO (/lio/odom), not FAST-LIO,
because Super-LIO's OctVoxMap is substantially more stable on z.
FAST-LIO2 still publishes /Odometry for visual comparison; toggle it
off with fastlio2:=false.

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
    fastlio2 = LaunchConfiguration('fastlio2')
    two_d = LaunchConfiguration('two_d')
    plot = LaunchConfiguration('plot')

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
        DeclareLaunchArgument(
            'fastlio2',
            default_value='true',
            description='Also launch FAST-LIO2 alongside Super-LIO for visual '
                        'comparison. EKF ignores it; set false to save CPU.',
        ),
        DeclareLaunchArgument(
            'two_d',
            default_value='true',
            description='Lock EKF z/roll/pitch to zero. Default true for flat '
                        'CARLA maps; set false for 3D scenarios or real vehicles.',
        ),
        DeclareLaunchArgument(
            'plot',
            default_value='false',
            description='Launch PlotJuggler alongside for real-time numeric '
                        'time-series of all pose sources. Load the bundled '
                        'layout via File > Load Layout (demos/plotjuggler_odom.xml).',
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

        # ── 2. Super-LIO (primary LIO, feeds the EKF) ────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'superlio.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'config_file': 'superlio_carla.yaml',
            }.items(),
        ),

        # ── 3. FAST-LIO2 (comparison only; EKF does not consume it) ─
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'fastlio2.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'config_file': 'fastlio2_carla.yaml',
            }.items(),
            condition=IfCondition(fastlio2),
        ),

        # ── 4. VINS-Fusion (off by default) ──────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'vins_fusion.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'config_file': 'vins_carla_mono.yaml',
            }.items(),
            condition=IfCondition(vins),
        ),

        # ── 5. EKF fusion + NavSat ───────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'ekf_fusion.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'two_d': two_d,
            }.items(),
        ),

        # ── 6. RViz2 (unified mapping + localization view) ───────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz),
        ),

        # ── 7. PlotJuggler (real-time numeric time-series) ───────
        Node(
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(plot),
        ),
    ])
