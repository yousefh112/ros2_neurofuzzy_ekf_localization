"""
Launch file for spawning a TurtleBot3 Burger robot in Gazebo.

This launch script performs the following operations:
1. Launches Gazebo with a custom indoor/outdoor world
2. Loads the robot description from URDF/XACRO files
3. Publishes the robot's TF tree using robot_state_publisher
4. Spawns the robot at a specified location in the Gazebo simulation
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate the launch description for the robot spawning process.
    
    Returns:
        LaunchDescription: A launch description containing all nodes and actions
                          needed to spawn and initialize the robot.
    """
    # Get the package directory for robot_description_pkg
    pkg_name = 'robot_description_pkg'
    pkg_dir = get_package_share_directory(pkg_name)

    # ============================================================================
    # 1. LAUNCH GAZEBO SIMULATOR
    # ============================================================================
    # Load the custom world file containing the indoor/outdoor environment.
    # The world file defines walls, floors, and outdoor areas for simulation.
    world_file = os.path.join(pkg_dir, 'worlds', 'indoor_outdoor.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # ============================================================================
    # 2. LOAD ROBOT DESCRIPTION
    # ============================================================================
    # Parse the XACRO file to generate the robot's URDF description.
    # The XACRO (XML Macros) file contains parameterized robot definitions
    # that are converted to standard URDF format at launch time.
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare(pkg_name),
                              'urdf', 'tb3_custom.urdf.xacro'])
    ])

    # Convert the command result to a string parameter for robot_state_publisher
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # ============================================================================
    # 3. PUBLISH ROBOT STATE AND TRANSFORMS
    # ============================================================================
    # Launch robot_state_publisher to:
    # - Read the robot description and create TF (transform) frames
    # - Publish the robot's state to the /tf topic
    # - Enable sim_time synchronization with Gazebo
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,  # Synchronize with Gazebo simulation time
        }]
    )

    # ============================================================================
    # 4. SPAWN ROBOT IN GAZEBO SIMULATION
    # ============================================================================
    # Spawn the robot model at the specified location in the Gazebo world.
    # 
    # World Coordinate System:
    #   - X-axis: [-10.86, 11.20] m (building envelope)
    #   - Y-axis: [-4.15, 4.55] m (building envelope)
    #
    # Spawn Location (5.0, 6.5):
    #   - Positioned ~1.95 m north of the building
    #   - Located in the OUTDOOR open grass area
    #
    # Z-axis Height (0.05 m):
    #   - Keeps the robot wheels just above ground level
    #   - Allows wheels to naturally snap to contact with the ground
    #   - Avoids large drops that would create initial angular impulse/drift
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',  # Use the robot description topic
            '-entity', 'tb3_burger',         # Entity name in Gazebo
            '-x', '5.0',                     # X position (meters)
            '-y', '6.5',                     # Y position (meters)
            '-z', '0.05',                    # Z position (height in meters)
        ],
        output='screen'
    )

    # ============================================================================
    # RETURN LAUNCH DESCRIPTION
    # ============================================================================
    # Combine all components into a single launch description.
    # The order matters: Gazebo should start first, followed by robot state
    # publisher, and finally the spawn operation.
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
