from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="pi_arm_websocket",
                executable="pi_arm_websocket",
                name="pi_arm_websocket",
                output="screen",
                parameters=[
                    {
                        "port": 8765,
                        "state_push_interval_ms": 100,
                        "max_size_bytes": 32 * 1024 * 1024,
                    }
                ],
            )
        ]
    )
