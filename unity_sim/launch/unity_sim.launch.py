from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene",
                default_value="ERC2023",
                description="The scene to load in Unity.",
            ),
            Node(
                package="unity_sim",
                executable="unity_sim",
                parameters=[
                    {"scene": LaunchConfiguration("scene")},
                ],
            ),
            # rosbridge server for C# <-> ROS communication
            Node(package="rosbridge_server", executable="rosbridge_websocket"),
            # custom Unix Socket server for camera image transfer
            Node(package="unity_rs_publisher", executable="unity_rs_publisher"),
            # tf_static republisher
            Node(
                package="unity_tf_static_republisher",
                executable="unity_tf_static_republisher",
            ),
        ]
    )
