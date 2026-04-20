#!/usr/bin/env python3
# launch/nav_launch.py
# ──────────────────────────────────────────────────────────────────────────────
# Run on JETSON AGX Xavier — Full Hardware Bringup + EKF + Nav2
#
# Stack:
#   1. robot_state_publisher      — URDF + static TF                   [t = 0.0s]
#   2. joint_state_publisher      — /joint_states                      [t = 1.5s]
#   3. wheel_odom_node            — STM32 UART → /odom                 [t = 0.0s]
#   4. bno055                     — I2C → /imu/data (~100 Hz)          [t = 0.0s]
#   5. imu_reader                 — Quaternion → /imu/euler            [t = 3.0s]
#   6. ekf_filter_node            — /odom + /imu/data → TF odom→base   [t = 7.0s]
#   7. sllidar_node               — RPLIDAR S2E UDP → /scan            [t = 0.0s]
#   8. scan_to_scan_filter_chain  — /scan → /scan_filtered             [t = 10.0s]
#   9. nav2_bringup               — AMCL + Planner + Controller + BT   [t = 12.0s]
#  10. checkpoint_navigator       — Autonomous waypoint sequencer      [t = 15.0s] (optional)
#
# Environment (Jetson + Laptop must be the same):
#   export ROS_DOMAIN_ID=42
#   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#
# Run local with default area (floor e6) :
#   ros2 launch mobile_robot nav_launch.py
#
# Run for a specific floor (e.g., e1) :
#   ros2 launch mobile_robot nav_launch.py floor:=e1
#
# Run with Checkpoint Navigator:
#   ros2 launch mobile_robot nav_launch.py autostart_navigator:=true
# ──────────────────────────────────────────────────────────────────────────────

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package & shared paths ─────────────────────────────────────────────────
    package_name     = 'mobile_robot'
    pkg_share        = get_package_share_directory(package_name)
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    urdf_file         = os.path.join(pkg_share, 'urdf',   'mobile_robot.urdf.xacro')
    ekf_config        = os.path.join(pkg_share, 'config', 'ekf.yaml')
    bno055_config     = os.path.join(pkg_share, 'config', 'bno055_params.yaml')
    wheel_odom_config = os.path.join(pkg_share, 'config', 'wheel_odom_params.yaml')
    filter_config     = os.path.join(pkg_share, 'config', 'laser_filter.yaml')
    nav2_params       = os.path.join(pkg_share, 'config', 'nav2_params_test.yaml')
    # default_map       = os.path.join(pkg_share, 'maps',   'lab_demo_map.yaml')
    session_logger    = os.path.join(pkg_share, 'scripts', 'session_logger.py')

    # ── Launch arguments ───────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true. Must be false on physical hardware.'
    )

    # declare_map = DeclareLaunchArgument(
    #     'map',
    #     default_value=default_map,
    #     description='Full path to the occupancy grid map .yaml file.'
    # )

    declare_floor = DeclareLaunchArgument(
        'floor',
        default_value='e6',
        description='Floor identifier (e.g., e6, e1)'
    )

    declare_autostart_navigator = DeclareLaunchArgument(
        'autostart_navigator',
        default_value='true',
        description='Automatically start the CheckpointNavigator node after Nav2 is ready.'
    )

    declare_timeout = DeclareLaunchArgument(
        'timeout_at_checkpoint',
        default_value='30.0',
        description='Dwell time in seconds at each checkpoint before returning to home.'
    )

    use_sim_time        = LaunchConfiguration('use_sim_time')
    # map_file            = LaunchConfiguration('map')
    floor = LaunchConfiguration('floor')
    autostart_navigator = LaunchConfiguration('autostart_navigator')
    timeout             = LaunchConfiguration('timeout_at_checkpoint')

    dynamic_map_file = [
        pkg_share, '/maps/map_', floor, '.yaml'
    ]

    dynamic_checkpoint_file = [
        pkg_share, '/config/checkpoints_', floor, '.yaml'
    ]

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 0 — Session Logger                                          [t = 0s]
    # Write structured log to ~/robot_logs/session_YYYY-MM-DD_HH-MM-SS.log
    # ══════════════════════════════════════════════════════════════════════════
    session_logger_node = Node(
        package='mobile_robot',
        executable='session_logger.py',
        name='session_logger',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # 'log_dir': os.path.expanduser('~/robot_logs'),
            'log_dir': os.path.join(
                os.path.expanduser('~'),
                'mbrobot_ws', 'src', 'mobile_robot', 'logs'
            ),
        }]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 1 — Robot State Publisher                                    [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': Command(['xacro ', urdf_file]),
            'publish_frequency': 50.0,
        }]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 2 — Joint State Publisher                                  [t = 1.5s]
    # ══════════════════════════════════════════════════════════════════════════
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    delayed_jsp = TimerAction(
        period=1.5,
        actions=[
            LogInfo(msg='[nav] [1.5s] Starting joint_state_publisher...'),
            joint_state_publisher,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 3 — Wheel Odometry Node (STM32 via UART)                    [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    # publish_tf = false (wheel_odom_params.yaml) — EKF handles TF odom→base_footprint
    wheel_odom_node = Node(
        package='mobile_robot',
        executable='wheel_odom_node',
        name='wheel_odom_node',
        output='screen',
        parameters=[wheel_odom_config],
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 4 — BNO055 IMU Driver (I2C bus 8, J23)                      [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    # Do NOT add inline params — avoids overriding yaml and losing ros_topic_prefix.
    # remapping: /imu/imu → /imu/data so EKF receives the correct topic.
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        parameters=[bno055_config],
        remappings=[
            ('/imu/imu', '/imu/data'),
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 5 — IMU Reader (Quaternion → Euler)                        [t = 3.0s]
    # ══════════════════════════════════════════════════════════════════════════
    imu_reader_node = Node(
        package='mobile_robot',
        executable='imu_reader',
        name='imu_reader',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    delayed_imu_reader = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[nav] [3.0s] Starting imu_reader...'),
            imu_reader_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 6 — EKF (robot_localization)                               [t = 7.0s]
    # ══════════════════════════════════════════════════════════════════════════
    # Fuses /odom + /imu/data → publishes TF odom → base_footprint
    # AMCL will publish TF map → odom (do NOT add a static_tf here)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    delayed_ekf = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg='[nav] [7.0s] Starting ekf_filter_node...'),
            ekf_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 7 — RPLIDAR S2E Driver (UDP)                                [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        respawn=True,               # ← tự restart khi chết
        respawn_delay=3.0,          # ← chờ 3s trước khi restart
        parameters=[{
            'channel_type':     'udp',
            'udp_ip':           '192.168.11.2',
            'udp_port':         8089,
            'frame_id':         'laser_frame',
            'inverted':         False,
            'angle_compensate': True,
            'scan_mode':        'Sensitivity',
            'use_sim_time':     use_sim_time,
        }]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 8 — Laser Scan Filter                                      [t = 10s]
    # ══════════════════════════════════════════════════════════════════════════
    scan_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[filter_config, {'use_sim_time': use_sim_time}]
    )

    delayed_scan_filter = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[nav] [10.0s] Starting scan_to_scan_filter_chain...'),
            scan_filter_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 9 — Nav2 Bringup                                           [t = 12s]
    # ══════════════════════════════════════════════════════════════════════════
    # Delay 12s = 10s (scan filter ready) + 2s buffer for /scan_filtered to stabilize.
    #
    # Launches the full Nav2 navigation stack:
    #   - map_server       : serves the pre-built occupancy grid map
    #   - amcl             : Monte Carlo localization on the known map
    #                        broadcasts TF map → odom
    #   - planner_server   : global path planning (NavFn / A*)
    #   - controller_server: local trajectory following (DWB)
    #   - bt_navigator     : behavior tree executive
    #   - recoveries_server: spin, back-up, wait recovery behaviors
    #
    # Config: nav2_params.yaml
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':          dynamic_map_file,
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params,
        }.items()
    )

    delayed_nav2 = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[nav] [12.0s] Starting Nav2 bringup...'),
            nav2_bringup,
        ]
    )

    set_initial_pose = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            '{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": -4.047, "y": -7.508, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.1701, "w": 0.9854}}, "covariance": [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.068]}}'
        ],
        output='screen'
    )

    delayed_initial_pose = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='[nav] [15.0s] Publishing initial pose to AMCL...'),
            set_initial_pose,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 10 — Checkpoint Navigator  (optional)                      [t = 15s]
    # ══════════════════════════════════════════════════════════════════════════
    # Only launched when autostart_navigator:=true.
    #
    # Control topics:
    #   /robot/navigate_to_checkpoint  (std_msgs/Int32  — input)
    #   /robot/emergency_stop          (std_msgs/Bool   — input)
    #   /robot/state                   (std_msgs/String — output)
    #   /robot/current_checkpoint      (std_msgs/Int32  — output)
    #   /robot/status_message          (std_msgs/String — output)
    checkpoint_navigator_node = Node(
        package='mobile_robot',
        executable='navigator.py',
        name='navigator',
        output='screen',
        condition=IfCondition(autostart_navigator),
        parameters=[{
            'use_sim_time':          use_sim_time,
            'checkpoint_file':       dynamic_checkpoint_file,
            'timeout_at_checkpoint': timeout,
            'home_checkpoint_id':    0,
        }]
    )

    delayed_checkpoint_nav = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='[nav] Starting checkpoint_navigator...'),
            checkpoint_navigator_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # Launch Description
    # ══════════════════════════════════════════════════════════════════════════
    return LaunchDescription([
        declare_use_sim_time,
        # declare_map,
        declare_floor,
        declare_autostart_navigator,
        declare_timeout,

        LogInfo(msg='[nav] ═══ Starting Full Hardware Bringup + EKF + Nav2 ═══'),
        LogInfo(msg='[nav] [0.0s] Starting RSP, wheel_odom, bno055, lidar...'),
        session_logger_node,
        robot_state_publisher,
        wheel_odom_node,
        bno055_node,
        lidar_node,

        delayed_jsp,
        delayed_imu_reader,
        delayed_ekf,
        delayed_scan_filter,
        delayed_nav2,
        delayed_initial_pose,
        delayed_checkpoint_nav,
    ])