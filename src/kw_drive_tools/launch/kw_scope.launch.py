from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kw_drive_tools",
            executable="kw_scope",
            name="kw_scope",
            output="screen",
            parameters=[{
                "topic": "/kw_motor/state",
                "window_sec": 5.0,
                "refresh_hz": 30.0,
            }],
        ),
    ])
