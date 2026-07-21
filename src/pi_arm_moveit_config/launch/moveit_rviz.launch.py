from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def _launch_setup(context):
    urdf_path = (
        get_package_share_directory("pi_arm_description")
        + "/urdf/pi_arm.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder("pi_arm", package_name="pi_arm_moveit_config")
        .robot_description(
            file_path=urdf_path,
            mappings={
                "backend": LaunchConfiguration("hardware_type").perform(context),
                "can_interface": LaunchConfiguration("can_interface").perform(context),
                "bitrate": LaunchConfiguration("bitrate").perform(context),
            },
        )
        .robot_description_semantic(file_path="config/pi_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )

    generated = generate_moveit_rviz_launch(moveit_config)
    return list(generated.entities)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hardware_type", default_value="mock", choices=["mock", "real"]
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("bitrate", default_value="1000000"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
