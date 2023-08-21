from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unity_sim',
            executable='simulation'
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket'
        ),
    ])
