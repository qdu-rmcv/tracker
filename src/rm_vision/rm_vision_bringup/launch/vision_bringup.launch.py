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


sys.path.append(
    os.path.join(get_package_share_directory("rm_vision_bringup"), "launch")
)


def _safe_taskset_prefix(cpu_list):
    """Bind a container only if the requested CPU list is valid on this NUC.

    This avoids launch failures on machines that do not have CPU7 or have not
    been tuned with CPU isolation. If taskset fails, the process is started
    normally.
    """
    cpu_list = str(cpu_list).strip()
    if not cpu_list:
        return None

    return (
        "bash -lc 'CPU_LIST=\"%s\"; "
        "if command -v taskset >/dev/null 2>&1 && "
        "taskset -c \"$CPU_LIST\" true >/dev/null 2>&1; then "
        "exec taskset -c \"$CPU_LIST\" \"$@\"; "
        "else "
        "echo \"[launch] skip taskset CPU_LIST=$CPU_LIST: unavailable or invalid on this machine\" >&2; "
        "exec \"$@\"; "
        "fi' --"
    ) % cpu_list


# 针对多台 NUC 的兼容优化：
# - 若 CPU4-6 / CPU7 存在，则按配置绑核；
# - 若某台 NUC 核心数不足或未做隔离，自动跳过 taskset，不影响启动；
# - 不再在 launch 里强制 chrt，SCHED_FIFO 由 planning_trajectory 内部实时线程尝试设置。
def _build_after_checkout(context, *args, **kwargs):
    from common import (
        launch_params,
        robot_state_publisher,
        get_camera_component,
        get_detector_component,
        get_tracker_component,
        get_trajectory_component,
        get_serial_component,
        get_marker_component,
    )

    robot_type = LaunchConfiguration("robot").perform(context)

    vision_cpu_list = launch_params.get("vision_cpu_list", "4-6")
    trajectory_cpu_list = launch_params.get("trajectory_cpu_list", "7")

    # 普通视觉节点：优先 CPU4-6；若机器不支持则自动跳过 taskset
    vision_container = ComposableNodeContainer(
        name="vision_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        prefix=_safe_taskset_prefix(vision_cpu_list),
        composable_node_descriptions=[
            get_camera_component(robot_type),
            get_detector_component(robot_type),
            get_tracker_component(robot_type),
            get_serial_component(robot_type),
            get_marker_component(robot_type),
        ],
        output="both",
        emulate_tty=True,
        parameters=[
            {"thread_num": os.cpu_count() - 2},
        ],
        ros_arguments=[
            "--ros-args",
            "--log-level",
            "armor_detector:=" + launch_params["detector_log_level"],
            "--log-level",
            "armor_tracker:=" + launch_params["tracker_log_level"],
            "--log-level",
            "serial_driver:=" + launch_params["serial_log_level"],
        ],
        on_exit=Shutdown(),
    )

    # trajectory 单独容器：优先 CPU7；SCHED_FIFO 在节点内部线程里设置
    trajectory_container = ComposableNodeContainer(
        name="trajectory_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        prefix=_safe_taskset_prefix(trajectory_cpu_list),
        composable_node_descriptions=[
            get_trajectory_component(robot_type),
        ],
        output="both",
        emulate_tty=True,
        parameters=[
            {"thread_num": 2},
        ],
        ros_arguments=[
            "--ros-args",
            "--log-level",
            "planning_trajectory:=" + launch_params.get("trajectory_log_level", "INFO"),
        ],
        on_exit=Shutdown(),
    )

    return [
        robot_state_publisher,
        vision_container,
        trajectory_container,
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
