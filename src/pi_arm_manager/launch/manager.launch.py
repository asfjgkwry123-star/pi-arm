from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="pi_arm_manager",
                executable="pi_arm_manager_node",
                name="pi_arm_manager",
                output="screen",
            )
        ]
    )
