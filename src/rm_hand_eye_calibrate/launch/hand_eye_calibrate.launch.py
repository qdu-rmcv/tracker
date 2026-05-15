import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory

sys.path.append(
    os.path.join(get_package_share_directory("rm_hand_eye_calibrate"), "launch")
)


def generate_launch_description():
    from launch_ros.actions import Node, ComposableNodeContainer
    from launch_ros.descriptions import ComposableNode
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription
    from launch.substitutions import Command

    node_params = os.path.join(
        get_package_share_directory("rm_hand_eye_calibrate"),
        "config",
        "node_params.yaml",
    )
    launch_params = yaml.safe_load(
        open(
            os.path.join(
                get_package_share_directory("rm_hand_eye_calibrate"),
                "config",
                "launch_params.yaml",
            )
        )
    )

    robot_description = Command(
        [
            "xacro ",
            os.path.join(
                get_package_share_directory("rm_gimbal_description"),
                "urdf",
                "rm_gimbal.urdf.xacro",
            ),
            " xyz:=",
            launch_params["odom2camera"]["xyz"],
            " rpy:=",
            launch_params["odom2camera"]["rpy"],
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "publish_frequency": 1000.0}
        ],
    )

    hik_camera_node = ComposableNode(
        package="hik_camera",
        plugin="HikCamera::HikCameraNode",
        name="camera_node",
        parameters=[node_params],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    hik_camera_container = ComposableNodeContainer(
        name="hik_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[hik_camera_node],
        output="both",
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    hand_eye_node = Node(
        package="rm_hand_eye_calibrate",
        executable="rm_hand_eye_calibrate_node",
        name="hand_eye_calibrate_node",
        output="both",
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
    )

    delay_hand_eye_node = TimerAction(
        period=2.0,
        actions=[hand_eye_node],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            hik_camera_container,
            delay_hand_eye_node,
        ]
    )
