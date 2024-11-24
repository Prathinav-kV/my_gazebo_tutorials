from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare an argument to enable/disable bag recording
    enable_bag_recording = LaunchConfiguration('enable_bag_recording')

    return LaunchDescription([
        # Declare argument to control bag recording
        DeclareLaunchArgument(
            'enable_bag_recording',
            default_value='true',
            description='Enable or disable rosbag recording'),

        # Start Gazebo with your custom world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
                 '/home/pratkv/walker_ws/src/walker/worlds/newest_2.world'],
            output='screen'
        ),

        # Spawn the TurtleBot3 Burger
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'turtlebot3_burger',
                 '-file', '/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf',
                 '-x', '0', '-y', '0', '-z', '0'],
            output='screen'
        ),

        # Launch the walker node
        ExecuteProcess(
            cmd=['ros2', 'run', 'walker', 'walker'],
            output='screen'
        ),

        # Record a bag file (only if enabled)
        ExecuteProcess(
            condition=IfCondition(enable_bag_recording),
            cmd=['ros2', 'bag', 'record', '-a', '--output', '/home/pratkv/walker_ws/results/walker_bag'],
            output='screen'
        )
    ])
