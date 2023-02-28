from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='woeden_agent',
            namespace='woeden_agent',
            executable='woeden_agent',
            name='agent'
        ),
        Node(
            package='woeden_agent',
            namespace='woeden_agent',
            executable='trigger_worker.py',
            name='trigger_worker'
        ),
        Node(
            package='woeden_agent',
            namespace='woeden_agent',
            executable='upload_worker.py',
            name='upload_worker'
        )
    ])