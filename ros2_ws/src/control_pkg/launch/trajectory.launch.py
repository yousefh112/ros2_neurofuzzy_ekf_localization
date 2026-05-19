from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('control_pkg')
    config_file = pkg_share + '/config/waypoints.yaml'

    return LaunchDescription([
        Node(
            package='control_pkg',
            executable='trajectory_controller',
            name='trajectory_controller',
            output='screen',
            parameters=[config_file],
        )
    ])
