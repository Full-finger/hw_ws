from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='w3hw_ros2py',
            executable='publisher_node',
            name='hello_world_publisher'
        ),
        Node(
            package='w3hw_ros2py',
            executable='subscriber_node',
            name='hello_world_subscriber'
        )
    ])
