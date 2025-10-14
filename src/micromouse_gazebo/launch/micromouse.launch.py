from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    pkg_gazebo = os.path.join(
        os.environ['HOME'], 'micromouse_ws', 'src', 'micromouse_gazebo'
    )

    world_path = os.path.join(pkg_gazebo, 'worlds', 'maze.world')
    robot_urdf = os.path.join(
        os.environ['HOME'], 'micromouse_ws', 'src', 'micromouse_description', 'urdf', 'micromouse.urdf.xacro'
    )

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn Robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'micromouse', '-file', robot_urdf],
            output='screen'
        ),
    ])
