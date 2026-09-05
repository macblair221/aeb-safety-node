from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ttc_threshold', default_value='1.0'),
        DeclareLaunchArgument('speed', default_value='2.0'),
        DeclareLaunchArgument('start_distance', default_value='10.0'),
        DeclareLaunchArgument('lateral_offset', default_value='0.0'),
        DeclareLaunchArgument('num_beams', default_value='181'),

        Node(
            package='safety_node',
            executable='safety_node',
            name='safety_node',
            parameters=[{
                'ttc_threshold': LaunchConfiguration('ttc_threshold'),
            }],
            output='screen',
        ),
        Node(
            package='test_publisher',
            executable='fake_sensors',
            name='fake_sensors',
            parameters=[{
                'speed': LaunchConfiguration('speed'),
                'start_distance': LaunchConfiguration('start_distance'),
                'lateral_offset': LaunchConfiguration('lateral_offset'),
                'num_beams': LaunchConfiguration('num_beams'),
            }],
            output='screen',
        ),
    ])