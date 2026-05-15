"""
sim_bringup.launch.py  —— 纯仿真 launch

组件容器内:
  - armor_simulator  (替代 camera + detector, 同时发布静态 TF)
  - armor_tracker
  - planning_trajectory
  - armor_marker

不启动:
  - camera / detector  (由 simulator 替代)
  - serial_driver      (无真实下位机)
  - robot_state_publisher (由 simulator 的静态 TF 替代, 云台角始终为零)

用法:
  ros2 launch rm_vision_bringup sim_bringup.launch.py
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

sys.path.append(
    os.path.join(get_package_share_directory("rm_vision_bringup"), "launch")
)


def generate_launch_description():
    from common import (
        launch_params,
        get_tracker_component,
        get_trajectory_component,
        get_marker_component,
    )

    robot_type = LaunchConfiguration("robot_type")

    node_params = os.path.join(
        get_package_share_directory("rm_vision_bringup"), "config", "node_params.yaml"
    )

    # ---- 仿真器组件 (替代 camera + detector, 并发布静态 TF) ----
    def get_simulator_component():
        return ComposableNode(
            package="rm_simulator",
            plugin="rm_auto_aim::RmSimulatorNode",
            name="rm_simulator",
            parameters=[
                node_params,
                {"publish_tf": True},  # 纯仿真模式: 仿真器负责 TF
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

    vision_container = ComposableNodeContainer(
        name="vision_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            get_simulator_component(),
            get_tracker_component(),
            get_trajectory_component(),
            get_marker_component(),
        ],
        output="both",
        emulate_tty=True,
        ros_arguments=[
            "--ros-args",
            "--log-level",
            "armor_tracker:=" + launch_params["tracker_log_level"],
            "--log-level",
            "planning_trajectory:=" + launch_params.get("trajectory_log_level", "INFO"),
        ],
        on_exit=Shutdown(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_type", default_value="default"),
            vision_container,
        ]
    )

