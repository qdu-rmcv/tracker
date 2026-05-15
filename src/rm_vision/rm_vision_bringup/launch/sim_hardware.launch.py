"""
sim_hardware_bringup.launch.py  —— 真云台 + 仿真装甲板 launch

组件容器内:
  - armor_simulator    (仅发布仿真装甲板, 不发布 TF)
  - armor_tracker
  - planning_trajectory
  - serial_driver      (真实下位机通信, 发布 joint_states)
  - armor_marker

额外节点:
  - robot_state_publisher  (通过 URDF + joint_states 发布 TF)

不启动:
  - camera / detector  (由 simulator 替代)

用法:
  ros2 launch rm_vision_bringup sim_hardware_bringup.launch.py robot:=<branch>
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

sys.path.append(
    os.path.join(get_package_share_directory("rm_vision_bringup"), "launch")
)


def _build_after_checkout(context, *args, **kwargs):
    from common import (
        launch_params,
        robot_state_publisher,
        get_tracker_component,
        get_trajectory_component,
        get_serial_component,
        get_marker_component,
    )

    robot_type = LaunchConfiguration("robot").perform(context)

    node_params = os.path.join(
        get_package_share_directory("rm_vision_bringup"), "config", "node_params.yaml"
    )

    # ---- 仿真器组件 (仅发布装甲板, 不发布 TF) ----
    simulator_component = ComposableNode(
        package="armor_simulator",
        plugin="rm_auto_aim::ArmorSimulatorNode",
        name="armor_simulator",
        parameters=[
            node_params,
            {
                "publish_tf": False,  # 真云台模式: TF 由 robot_state_publisher 负责
                "robot_type": robot_type,
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    vision_container = ComposableNodeContainer(
        name="vision_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            simulator_component,
            get_tracker_component(robot_type),
            get_trajectory_component(robot_type),
            get_serial_component(robot_type),
            get_marker_component(robot_type),
        ],
        output="both",
        emulate_tty=True,
        ros_arguments=[
            "--ros-args",
            "--log-level",
            "armor_tracker:=" + launch_params["tracker_log_level"],
            "--log-level",
            "planning_trajectory:=" + launch_params.get("trajectory_log_level", "INFO"),
            "--log-level",
            "serial_driver:=" + launch_params["serial_log_level"],
        ],
        on_exit=Shutdown(),
    )

    return [
        robot_state_publisher,
        vision_container,
    ]


def generate_launch_description():
    ws_root = LaunchConfiguration("ws_root")
    robot = LaunchConfiguration("robot")

    config_repo_rel = "src/rm_vision/rm_vision_bringup/config"

    checkout_robot = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "set -e; "
            'cd "$WS_ROOT"/' + config_repo_rel + "; "
            "git rev-parse --is-inside-work-tree >/dev/null 2>&1; "
            'git checkout "$BRANCH"; '
            "git status --porcelain",
        ],
        additional_env={
            "WS_ROOT": ws_root,
            "BRANCH": robot,
        },
        output="screen",
    )

    build_nodes = OpaqueFunction(function=_build_after_checkout)

    return LaunchDescription(
        [
            DeclareLaunchArgument("ws_root", default_value=os.getcwd()),
            DeclareLaunchArgument("robot", default_value=""),
            checkout_robot,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=checkout_robot,
                    on_exit=[build_nodes],
                )
            ),
        ]
    )

