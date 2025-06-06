import os

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Build MoveIt2 configuration for your robot
    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    # 1) Static transform publisher for camera → base_link
    static_tf = Node(
        package="object_detection",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        # (no extra parameters—static transform values are hardcoded in the node)
    )

    # 2) Perception node: subscribes to the depth‐camera topic and publishes /object_detected
    object_detection_node = Node(
        package="object_detection",
        executable="object_detection",
        name="object_detection",
        output="screen",
    )

    # 3) MoveItCPP‐based pick‐and‐place that waits on /object_detected
    pick_and_place_node = Node(
        name="pick_and_place",
        package="moveit2_scripts",
        executable="pick_and_place_perception",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        static_tf,
        object_detection_node,
        pick_and_place_node
    ])
