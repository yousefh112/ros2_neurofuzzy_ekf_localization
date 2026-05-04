import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Package paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('robot_description_pkg')

    # 2. Define the direct path to the .world file
    # Ensure the path resolves correctly to Pietro's indoor/outdoor environment
    world_path = os.path.join(pkg_robot_description, 'worlds', 'indoor_outdoor.world')

    # DEBUG: Print the world path to the terminal during launch
    print(f"\n--- LOADING WORLD: {world_path} ---\n")

    # 3. Include Gazebo, passing the world path string directly
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        # Pass world_path directly instead of using LaunchConfiguration
        launch_arguments={'world': world_path}.items()
    )

    return LaunchDescription([
        gazebo
    ])