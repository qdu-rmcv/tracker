import os
import sys
from typing import Any, cast

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

sys.path.append(
    os.path.join(get_package_share_directory("rm_vision_bringup"), "launch")
)


def get_video_detector_container(
    context, *args, **kwargs
) -> list[ComposableNodeContainer]:
    from common import node_params, launch_params

    video_path = LaunchConfiguration("video_path").perform(context)
    loop = LaunchConfiguration("loop").perform(context)
    playback_rate = LaunchConfiguration("playback_rate").perform(context)
    publish_fps = LaunchConfiguration("publish_fps").perform(context)

    video_publisher_params = cast(list[str | dict[str, Any]], [node_params])
    override_params: dict[str, Any] = {}

    if video_path != "":
        override_params["video_path"] = video_path
    if loop != "":
        override_params["loop"] = ParameterValue(
            LaunchConfiguration("loop"), value_type=bool
        )
    if playback_rate != "":
        override_params["playback_rate"] = ParameterValue(
            LaunchConfiguration("playback_rate"), value_type=float
        )
    if publish_fps != "":
        override_params["publish_fps"] = ParameterValue(
            LaunchConfiguration("publish_fps"), value_type=float
        )

    if override_params:
        video_publisher_params.append(override_params)

    video_publisher_node = ComposableNode(
        package="video_publisher",
        plugin="VideoPublisherNode",
        name="video_publisher",
        parameters=video_publisher_params,
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    armor_detector_node = ComposableNode(
        package="armor_detector",
        plugin="rm_auto_aim::ArmorDetectorNode",
        name="armor_detector",
        parameters=[node_params],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name="video_detector_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            video_publisher_node,
            armor_detector_node,
        ],
        output="both",
        emulate_tty=True,
        ros_arguments=[
            "--ros-args",
            "--log-level",
            "video_publisher:=INFO",
            "--log-level",
            "armor_detector:=" + launch_params["detector_log_level"],
        ],
        on_exit=Shutdown(),
    )

    return [container]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "video_path",
                default_value="",
                description=(
                    "Optional path to the video file. "
                    "If empty, use video_path from node_params."
                ),
            ),
            DeclareLaunchArgument(
                "loop",
                default_value="",
                description=(
                    "Optional override for loop. "
                    "If empty, use loop from node_params."
                ),
            ),
            DeclareLaunchArgument(
                "playback_rate",
                default_value="",
                description=(
                    "Optional override for playback_rate. "
                    "If empty, use playback_rate from node_params."
                ),
            ),
            DeclareLaunchArgument(
                "publish_fps",
                default_value="",
                description=(
                    "Optional override for publish_fps. "
                    "If empty, use publish_fps from node_params."
                ),
            ),
            OpaqueFunction(function=get_video_detector_container),
        ]
    )
