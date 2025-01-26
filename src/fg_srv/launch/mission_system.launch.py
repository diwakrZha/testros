#src/fg_srv/launch/mission_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the URDF file
    urdf_path = os.path.join(
        get_package_share_directory('fg_srv'),
        'urdf',
        'mission_robot.urdf'
    )

    # Get the path to the world file (create an empty world if not exists)
    world_path = os.path.join(
        get_package_share_directory('fg_srv'),
        'worlds',
        'empty.world'
    )

    # If world file doesn't exist, create a simple empty world
    os.makedirs(os.path.dirname(world_path), exist_ok=True)
    if not os.path.exists(world_path):
        with open(world_path, 'w') as f:
            f.write('''
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
''')

    return LaunchDescription([
        # REST API Node
        Node(
            package='fg_srv',
            executable='mission_api',
            name='mission_api_node',
            output='screen'
        ),
        # Mission Retrieval Node
        Node(
            package='fg_srv',
            executable='mission_retrieval_node',
            name='mission_retrieval_node',
            output='screen'
        ),
        # Mission Subscriber Node
        Node(
            package='fg_srv',
            executable='mission_subscriber_node',
            name='mission_subscriber_node',
            output='screen'
        ),
        # Mission Robot Controller Node
        Node(
            package='fg_srv',
            executable='mission_robot_controller',
            name='mission_robot_controller_node',
            output='screen'
        ),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf_path]
        ),
        # Gazebo Simulation
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, 
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # URDF Spawner
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'mission_robot', 
                '-file', urdf_path,
                '-x', '0', 
                '-y', '0', 
                '-z', '1'  # Slightly above ground
            ],
            output='screen'
        )
    ])