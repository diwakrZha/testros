# /src/fg_srv/launch/mission_simulation.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the URDF file
    urdf_path = os.path.join(
        get_package_share_directory('fg_srv'),
        'urdf',
        'mission_robot.urdf'
    )

    # Gazebo launch
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # URDF spawner
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mission_robot', 
            '-file', urdf_path,
            '-x', '0', 
            '-y', '0', 
            '-z', '0'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        arguments=[urdf_path]
    )

    # Mission coordinate subscriber node
    mission_subscriber = Node(
        package='fg_srv',
        executable='mission_subscriber_node',
        name='mission_robot_controller'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        mission_subscriber
    ])