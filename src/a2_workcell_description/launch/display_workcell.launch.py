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
    rviz_config = package_share / "rviz" / "workcell.rviz"

    robot_description = xacro.process_file(
        str(xacro_file)
    ).toxml()

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="workcell_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": False,
                    }
                ],
                remappings=[
                    ("robot_description", "/workcell_description"),
                ],
            ),

            Node(
                package="rviz2",
                executable="rviz2",
                name="workcell_rviz",
                output="screen",
                #arguments=["-d",str(rviz_config)],
            ),
        ]
    )