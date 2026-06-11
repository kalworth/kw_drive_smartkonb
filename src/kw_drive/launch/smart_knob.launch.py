from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = Path(get_package_share_directory("kw_drive")) / "config" / "smart_knob.yaml"

    return LaunchDescription([
        Node(
            package="kw_drive",
            executable="kw_smart_knob",
            name="kw_smart_knob",
            output="screen",
            parameters=[str(config_file)],
        ),
    ])
