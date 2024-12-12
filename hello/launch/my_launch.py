from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hello',
            namespace='hello',
            executable='talker',
        ),
        Node(
            package='hello',
            namespace='hello',
            executable='listener',
        ),
    ])