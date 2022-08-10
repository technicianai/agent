from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='woden_monitor',
            namespace='woden_monitor',
            executable='alive',
            name='mqtt_alive_publisher'
        )
    ])