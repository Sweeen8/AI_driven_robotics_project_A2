from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    workcell_share = Path(
        get_package_share_directory("a2_workcell_description")
    )

    xarm_moveit_share = Path(
        get_package_share_directory("xarm_moveit_config")
    )

    robot_ip_argument = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.1.155",
        description="IP address of the physical UFACTORY Lite 6",
    )

    robot_ip = LaunchConfiguration("robot_ip")

    # Starts:
    # - workcell_state_publisher
    # - /workcell_description
    # - environment_root TF tree
    # - robot_mount_frame -> world transform
    workcell_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                workcell_share
                / "launch"
                / "display_workcell.launch.py"
            )
        )
    )

    # Starts:
    # - connection to the physical Lite 6
    # - ros2_control
    # - MoveIt move_group
    # - robot_state_publisher
    # - interactive MoveIt RViz
    lite6_real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                xarm_moveit_share
                / "launch"
                / "lite6_moveit_realmove.launch.py"
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
        }.items(),
    )

    return LaunchDescription(
        [
            robot_ip_argument,

            # Start the workcell TF tree first.
            workcell_launch,

            # Give the workcell frames time to become available.
            TimerAction(
                period=1.0,
                actions=[
                    lite6_real_launch,
                ],
            ),
        ]
    )
