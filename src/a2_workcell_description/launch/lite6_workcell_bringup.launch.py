from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)


def generate_launch_description() -> LaunchDescription:
    workcell_share = Path(
        get_package_share_directory("a2_workcell_description")
    )

    xarm_moveit_share = Path(
        get_package_share_directory("xarm_moveit_config")
    )

    rviz_config_file = (
        workcell_share
        / "rviz"
        / "workcell_klaar.rviz"
    )

    workcell_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                workcell_share
                / "launch"
                / "display_workcell.launch.py"
            )
        )
    )

    lite6_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                xarm_moveit_share
                / "launch"
                / "lite6_moveit_fake.launch.py"
            )
        )
    )

    return LaunchDescription(
        [
            workcell_launch,

            TimerAction(
                period=1.0,
                actions=[
                    lite6_moveit_launch,
                ],
            ),
        ]
    )