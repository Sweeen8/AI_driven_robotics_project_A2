from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    workcell_share = Path(
        get_package_share_directory("a2_workcell_description")
    )

    xarm_moveit_share = Path(
        get_package_share_directory("xarm_moveit_config")
    )

    # --- Workcell environment (table, platform, camera stand, etc.) ----------
    workcell_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(workcell_share / "launch" / "display_workcell.launch.py")
        )
    )

    # --- Lite6 + MoveIt + built-in gripper ----------------------------------
    # _robot_moveit_fake.launch.py is the parameterised entry point.
    # add_gripper=true attaches the UFLite two-finger gripper to link6 so it
    # appears in RViz and is part of the MoveIt planning model.
    lite6_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(xarm_moveit_share / "launch" / "_robot_moveit_fake.launch.py")
        ),
        launch_arguments={
            "dof": "6",
            "robot_type": "lite",
            "add_gripper": "true",
            "hw_ns": "ufactory",
            "no_gui_ctrl": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            workcell_launch,

            # MoveIt starts after workcell so TF root is already present.
            TimerAction(
                period=1.0,
                actions=[lite6_moveit_launch],
            ),
        ]
    )
