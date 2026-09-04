from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safety_node',
            executable='safety_node',
            name='safety_node',
            parameters=[{'ttc_threshold': 1.0}],
            output='screen',
        ),
        Node(
            package='test_publisher',
            executable='fake_sensors',
            name='fake_sensors',
            parameters=[{'speed': 2.0, 'start_distance': 10.0}],
            output='screen',
        ),
    ])