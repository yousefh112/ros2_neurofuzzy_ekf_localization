from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_nodes',
            executable='c1_node',
            name='c1_node',
            output='screen',
        ),
        Node(
            package='control_nodes',
            executable='c2_node',
            name='c2_node',
            output='screen',
        ),
    ])
