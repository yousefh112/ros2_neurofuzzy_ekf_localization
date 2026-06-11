"""
Launch file to load and run Gazebo with the indoor/outdoor world.

This launch file initializes the Gazebo simulator with a predefined world
environment. It retrieves the world file from the robot_description_pkg
and passes it to the gazebo_ros launch configuration.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Generate the launch description for Gazebo with world environment.
    
    Returns:
        LaunchDescription: Contains the Gazebo launch configuration with
                          the indoor/outdoor world file.
    """
    # Retrieve package share directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('robot_description_pkg')

    # Construct the full path to the world file
    world_path = os.path.join(pkg_robot_description, 'worlds', 'indoor_outdoor.world')

    # Print the world path for debugging purposes
    print(f"\n--- LOADING WORLD: {world_path} ---\n")

    # Create the Gazebo launch description with the world file argument
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Return the launch description containing Gazebo
    return LaunchDescription([
        gazebo
    ])