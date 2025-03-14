from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode


def launch_setup(context):
    component_container = (
        LaunchConfiguration("component_container").perform(context).strip()
    )
    scene = LaunchConfiguration("scene").perform(context)
    selective_launch = LaunchConfiguration("selective_launch").perform(context)

    description = []

    if selective_launch != "only_rs_pub":
        description += [
            Node(
                package="unity_sim",
                executable="unity_sim",
                parameters=[
                    {"scene": scene},
                ],
            ),
            # rosbridge server for C# <-> ROS communication
            Node(package="rosbridge_server", executable="rosbridge_websocket"),
            # tf_static republisher
            # ROSBridge has problems with transient local QOS,
            # so we continuously republish the static TFs
            Node(
                package="unity_tf_static_republisher",
                executable="unity_tf_static_republisher",
            ),
        ]

    if selective_launch != "no_rs_pub":
        # custom Unix Socket server for camera image transfer
        if component_container:
            description.append(
                LoadComposableNodes(
                    target_container=component_container,
                    composable_node_descriptions=[
                        ComposableNode(
                            package="unity_rs_publisher",
                            plugin="unity_rs_publisher::UnityRsPublisher",
                            extra_arguments=[{"use_intra_process_comms": True}],
                        ),
                    ],
                )
            )
        else:
            description += [
                Node(package="unity_rs_publisher", executable="unity_rs_publisher"),
            ]

    return description


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "component_container",
                default_value="",
                description="Name of an existing component container to use. Empty to disable composition.",
            ),
            DeclareLaunchArgument(
                "scene",
                default_value="ERC2023",
                description="The scene to load in Unity.",
            ),
            DeclareLaunchArgument(
                "selective_launch",
                default_value="all",
                choices=["all", "no_rs_pub", "only_rs_pub"],
                description="Selectively launch Unity and/or camera publisher.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
