from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # ==================== Paths ====================
    pkg_gazebo = os.path.join(
        os.environ['HOME'], 'micromouse_ws', 'src', 'micromouse_gazebo'
    )
    world_path = os.path.join(pkg_gazebo, 'worlds', 'maze.world')

    # Robot description via xacro
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('micromouse_description'),
            'urdf',
            'micromouse.urdf.xacro'
        ])
    ])

    # ==================== Nodes ====================
    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Publish robot description to /robot_description
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn robot after a small delay to ensure Gazebo is ready
    spawn_robot = TimerAction(
        period=3.0,  # wait 3 seconds
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'micromouse', '-topic', '/robot_description'],
            output='screen'
        )]
    )

    # ==================== Launch Description ====================
    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_robot
    ])
