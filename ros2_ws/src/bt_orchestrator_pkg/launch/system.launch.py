import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def get_pkg(name: str) -> str:
    return get_package_share_directory(name)


def generate_launch_description() -> LaunchDescription:

    pkg_robot_desc   = get_pkg('robot_description_pkg')
    pkg_gps_ins      = get_pkg('gps_ins_pkg')
    pkg_gps_odom     = get_pkg('gps_odometry_pkg')
    pkg_control      = get_pkg('control_pkg')
    pkg_bt           = get_pkg('bt_orchestrator_pkg')

    cfg_ekf1         = os.path.join(pkg_gps_ins,  'config', 'ekf_1.yaml')
    cfg_ekf2         = os.path.join(pkg_gps_odom, 'config', 'ekf_2.yaml')
    cfg_waypoints    = os.path.join(pkg_control,  'config', 'waypoints.yaml')

    args = [
        DeclareLaunchArgument('use_rviz',     default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('ekf_delay',    default_value='8.0',
            description='Wait before starting EKF nodes'),
        DeclareLaunchArgument('fusion_delay', default_value='12.0',
            description='Wait before starting fusion/intelligence nodes'),
        DeclareLaunchArgument('bt_delay',     default_value='9.0',
            description='Wait before starting BT brain (must be > ekf_delay)'),
        DeclareLaunchArgument('log_level',    default_value='INFO'),
    ]

    use_rviz     = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_delay    = LaunchConfiguration('ekf_delay')
    fusion_delay = LaunchConfiguration('fusion_delay')
    bt_delay     = LaunchConfiguration('bt_delay')
    log_level    = LaunchConfiguration('log_level')

    sim_time_param = {'use_sim_time': use_sim_time}

    # PHASE 1: Simulation (t=0s)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_desc, 'launch', 'spawn_robot.launch.py')
        )
    )

    # PHASE 2: Localization stack (t=ekf_delay)
    ekf1_local = Node(
        package='robot_localization', executable='ekf_node', name='ekf_local',
        output='screen', parameters=[cfg_ekf1, sim_time_param],
        remappings=[('odometry/filtered', '/odometry/local')],
        arguments=['--ros-args', '--log-level', log_level],
    )

    ekf1_global = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node_gps_imu', output='screen',
        parameters=[cfg_ekf1, sim_time_param],
        remappings=[('odometry/filtered', '/odometry/global')],
        arguments=['--ros-args', '--log-level', log_level],
    )

    navsat1 = Node(
        package='robot_localization', executable='navsat_transform_node',
        name='navsat_transform', output='screen',
        parameters=[cfg_ekf1, sim_time_param],
        remappings=[
            ('imu',               '/imu/data'),
            ('gps/fix',           '/gps/fix'),
            ('gps/filtered',      '/gps/filtered'),
            ('odometry/gps',      '/odometry/gps'),
            ('odometry/filtered', '/odometry/global'),
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    ekf2_local = Node(
        package='robot_localization', executable='ekf_node', name='ekf_local2',
        output='screen', parameters=[cfg_ekf2, sim_time_param],
        remappings=[('odometry/filtered', '/odometry/local2')],
        arguments=['--ros-args', '--log-level', log_level],
    )

    ekf2_global = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node_gps_enc', output='screen',
        parameters=[cfg_ekf2, sim_time_param],
        remappings=[('odometry/filtered', '/odometry/global2')],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # navsat_transform2 intentionally removed: two instances writing to the same
    # /odometry/gps topic created a race condition corrupting GPS→ENU for both EKFs.
    # A single navsat_transform (fed by EKF1's odometry for heading) is correct —
    # GPS hardware is one physical sensor whose ENU projection is EKF-independent.

    phase2 = TimerAction(
        period=ekf_delay,
        actions=[
            LogInfo(msg='[PHASE 2] Starting EKF1 + EKF2 stacks'),
            ekf1_local, ekf1_global, navsat1,
            ekf2_local, ekf2_global,
        ]
    )

    # PHASE 3: Fusion + Intelligence (t=fusion_delay)
    complementary_filter = Node(
        package='gps_ins_pkg', executable='complementary_filter',
        name='complementary_filter_node', output='screen',
        parameters=[sim_time_param],
        arguments=['--ros-args', '--log-level', log_level],
    )

    ann_node = Node(
        package='robot_control_brain', executable='trajectory_nn_node',
        name='online_training_node', output='screen',
        parameters=[sim_time_param],
        arguments=['--ros-args', '--log-level', log_level],
    )

    trajectory_controller = Node(
        package='control_pkg', executable='trajectory_controller',
        name='trajectory_controller', output='screen',
        parameters=[cfg_waypoints, sim_time_param],
        arguments=['--ros-args', '--log-level', log_level],
    )

    rviz_config = os.path.join(pkg_bt, 'rviz', 'system.rviz')
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[sim_time_param],
        condition=IfCondition(use_rviz),
    )

    phase3 = TimerAction(
        period=fusion_delay,
        actions=[
            LogInfo(msg='[PHASE 3] Starting Fusion + ANN + Trajectory Control'),
            complementary_filter, ann_node, trajectory_controller, rviz,
        ]
    )

    # PHASE 4: BT Brain (t=bt_delay)
    bt_brain = Node(
        package='bt_orchestrator_pkg', executable='bt_brain', name='bt_brain',
        output='screen', parameters=[sim_time_param],
        arguments=['--ros-args', '--log-level', log_level],
    )

    phase4 = TimerAction(
        period=bt_delay,
        actions=[
            LogInfo(msg='[PHASE 4] Starting BT Brain'),
            bt_brain,
        ]
    )

    # Gazebo publishes /clock at 10 Hz by default; EKF sim-time timers stall below
    # 30 Hz. Setting publish_rate=100 via ros2 param is the only reliable fix in
    # Humble (passing --ros-args to gzserver directly crashes it).
    set_clock_rate = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/gazebo', 'publish_rate', '100.0'],
                output='screen',
            )
        ]
    )

    return LaunchDescription(
        args + [
            LogInfo(msg='[SYSTEM] GPS/INS/Odometer Fusion starting'),
            simulation,
            set_clock_rate,
            phase2,
            phase3,
            phase4,
        ]
    )
