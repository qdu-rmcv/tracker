import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name = 'rm_serial_driver'
    share_dir = get_package_share_directory(package_name)
    params_file = os.path.join(
        share_dir,
        'config',
        'serial.yaml'
    )
    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='rm_serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[params_file]
    )

    return LaunchDescription([
        serial_driver_node
    ])
