import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_bringup')
    
    ekf_gps_imu = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_gps_imu',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'GPS_INS.yaml'), {'use_sim_time': True}],
        remappings=[('/odometry/filtered', '/odometry/filtered_1')]
    )

    ekf_gps_enc = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_gps_enc',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'GPS_odometer.yaml'), {'use_sim_time': True}],
        remappings=[('/odometry/filtered', '/odometry/filtered_2')]
    )

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'GPS_INS.yaml'), {'use_sim_time': True}],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odom') # Use Gazebo Odom to jumpstart GPS
        ]
    )

    comp_filter = Node(
        package='robot_bringup',
        executable='complementary_filter', 
        name='complementary_filter_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        ekf_gps_imu,
        ekf_gps_enc,
        navsat_transform,
        comp_filter
    ])