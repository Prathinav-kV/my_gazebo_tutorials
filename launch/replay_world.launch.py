from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo with the custom world
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
                '/home/pratkv/walker_ws/src/my_gazebo_tutorials/worlds/newest_2.world'
            ],
            output='screen'
        ),

        # Spawn the TurtleBot3 in the custom world
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_burger',
                '-file', '/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf',
                '-x', '0', '-y', '0', '-z', '0'
            ],
            output='screen'
        ),
    ])
