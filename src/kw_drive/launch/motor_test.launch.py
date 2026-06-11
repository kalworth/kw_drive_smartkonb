from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    config_file = Path(get_package_share_directory("kw_drive")) / "config" / "motor_test.yaml"

    return LaunchDescription([
        Node(
            package="kw_drive",
            executable="kw_motor_test",
            name="kw_motor_test",
            output="screen",
            parameters=[str(config_file)],
        ),
    ])
