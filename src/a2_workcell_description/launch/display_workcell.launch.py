from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(
        get_package_share_directory("a2_workcell_description")
    )

    xacro_file = package_share / "urdf" / "workcell.urdf.xacro"

    workcell_description = xacro.process_file(
        str(xacro_file)
    ).toxml()

    workcell_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="workcell_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": workcell_description,
                "use_sim_time": False,
            }
        ],
        remappings=[
            (
                "robot_description",
                "/workcell_description",
            ),
        ],
    )

    mount_lite6_to_workcell = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mount_lite6_to_workcell",
        output="screen",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "-1.5708",
            "--frame-id", "robot_mount_frame",
            "--child-frame-id", "world",
        ],
    )

    return LaunchDescription(
        [
            workcell_state_publisher,
            mount_lite6_to_workcell,
        ]
    )