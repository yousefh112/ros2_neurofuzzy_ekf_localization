# Standalone EKF2 launch (ekf_local2 + ekf_filter_node_gps_enc + navsat_transform2).
# Not used by the system — use bt_orchestrator_pkg/launch/system.launch.py instead.
# WARNING: Do NOT run this directly alongside system.launch.py. navsat_transform2
# writes to /odometry/gps and races with navsat_transform, corrupting GPS→ENU for both EKFs.

from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_localization_dir = get_package_share_directory('gps_odometry_pkg')
    parameters_file_dir = os.path.join(robot_localization_dir, 'config')
    parameters_file_path = os.path.join(parameters_file_dir, 'ekf_2.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'output_location',
	    default_value='~/dual_ekf_navsat_example_debug.txt'),
	
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_local2',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', '/odometry/local2')]           
           ),
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_gps_enc',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', '/odometry/global2')]
           ),           
    launch_ros.actions.Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform2',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('imu', '/imu/data'),
                        ('gps/fix', '/gps/fix'), 
                        ('gps/filtered', '/gps/filtered'),
                        ('odometry/gps', '/odometry/gps'),
                        ('odometry/filtered', '/odometry/global2')]           
           )           
])