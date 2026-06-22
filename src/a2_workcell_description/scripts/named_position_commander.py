#!/usr/bin/env python3

"""Plan and execute predefined Lite 6 joint positions through MoveIt."""

from pathlib import Path
from typing import Dict, List

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
)
from rclpy.action import ActionClient
from rclpy.node import Node


class NamedPositionCommander(Node):
    """Send named joint configurations to MoveIt."""

    def __init__(self) -> None:
        super().__init__("named_position_commander")

        self.declare_parameter("target", "home")
        self.declare_parameter("execute", True)
        self.declare_parameter("planning_time", 10.0)
        self.declare_parameter("planning_attempts", 10)
        self.declare_parameter("velocity_scaling", 0.15)
        self.declare_parameter("acceleration_scaling", 0.15)

        self.target_name = str(
            self.get_parameter("target").value
        )

        self.execute = bool(
            self.get_parameter("execute").value
        )

        self.planning_time = float(
            self.get_parameter("planning_time").value
        )

        self.planning_attempts = int(
            self.get_parameter("planning_attempts").value
        )

        self.velocity_scaling = float(
            self.get_parameter("velocity_scaling").value
        )

        self.acceleration_scaling = float(
            self.get_parameter("acceleration_scaling").value
        )

        self.config = self.load_configuration()

        self.planning_group = str(
            self.config["planning_group"]
        )

        self.joint_names: List[str] = list(
            self.config["joint_names"]
        )

        self.positions: Dict[str, dict] = dict(
            self.config["positions"]
        )

        if self.target_name not in self.positions:
            valid_positions = ", ".join(self.positions.keys())

            raise ValueError(
                f"Unknown target '{self.target_name}'. "
                f"Available positions: {valid_positions}"
            )

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
        )

        self.timer = self.create_timer(
            0.5,
            self.start_command,
        )

        self.command_started = False

        self.get_logger().info(
            f"Selected named position: {self.target_name}"
        )

    @staticmethod
    def load_configuration() -> dict:
        """Load the named-position configuration from the package."""

        package_share = Path(
            get_package_share_directory(
                "a2_workcell_description"
            )
        )

        config_path = (
            package_share
            / "config"
            / "named_positions.yaml"
        )

        if not config_path.exists():
            raise FileNotFoundError(
                f"Configuration file not found: {config_path}"
            )

        with config_path.open(
            "r",
            encoding="utf-8",
        ) as config_file:
            config = yaml.safe_load(config_file)

        if not isinstance(config, dict):
            raise ValueError(
                "named_positions.yaml does not contain "
                "a valid dictionary."
            )

        return config

    def start_command(self) -> None:
        """Wait for MoveIt and send the selected command once."""

        if self.command_started:
            return

        if not self.move_group_client.wait_for_server(
            timeout_sec=0.1
        ):
            self.get_logger().info(
                "Waiting for MoveIt action server /move_action..."
            )
            return

        self.command_started = True
        self.timer.cancel()

        goal_message = self.create_move_group_goal()

        self.get_logger().info(
            f"Sending target '{self.target_name}' "
            f"to planning group '{self.planning_group}'."
        )

        send_goal_future = (
            self.move_group_client.send_goal_async(
                goal_message,
                feedback_callback=self.feedback_callback,
            )
        )

        send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def create_move_group_goal(self) -> MoveGroup.Goal:
        """Create a MoveGroup action goal using joint constraints."""

        target_data = self.positions[self.target_name]
        joint_values = list(target_data["joints_rad"])

        if len(joint_values) != len(self.joint_names):
            raise ValueError(
                f"Position '{self.target_name}' has "
                f"{len(joint_values)} values, but "
                f"{len(self.joint_names)} joint names were configured."
            )

        constraints = Constraints()
        constraints.name = self.target_name

        for joint_name, joint_value in zip(
            self.joint_names,
            joint_values,
        ):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(joint_value)

            # About 0.57 degrees tolerance.
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0

            constraints.joint_constraints.append(
                joint_constraint
            )

        request = MotionPlanRequest()
        request.group_name = self.planning_group
        request.num_planning_attempts = self.planning_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = (
            self.velocity_scaling
        )
        request.max_acceleration_scaling_factor = (
            self.acceleration_scaling
        )
        request.goal_constraints.append(constraints)

        planning_options = PlanningOptions()

        # False means MoveIt should execute after planning.
        planning_options.plan_only = not self.execute
        planning_options.look_around = False
        planning_options.replan = True
        planning_options.replan_attempts = 3
        planning_options.replan_delay = 0.5

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = planning_options

        return goal

    def feedback_callback(
        self,
        feedback_message,
    ) -> None:
        """Print MoveIt's current processing state."""

        state = feedback_message.feedback.state

        self.get_logger().info(
            f"MoveIt state: {state}"
        )

    def goal_response_callback(self, future) -> None:
        """Handle acceptance or rejection of the action goal."""

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                "MoveIt rejected the named-position goal."
            )

            rclpy.shutdown()
            return

        self.get_logger().info(
            "Goal accepted. Waiting for planning/execution result."
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future) -> None:
        """Handle the completed MoveIt result."""

        action_result = future.result()
        result = action_result.result

        error_code = result.error_code.val

        if error_code == result.error_code.SUCCESS:
            operation = (
                "planned and executed"
                if self.execute
                else "planned"
            )

            self.get_logger().info(
                f"Position '{self.target_name}' "
                f"was successfully {operation}."
            )
        else:
            self.get_logger().error(
                f"MoveIt failed for position "
                f"'{self.target_name}'. "
                f"Error code: {error_code}"
            )

        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)

    try:
        node = NamedPositionCommander()
    except (
        FileNotFoundError,
        KeyError,
        TypeError,
        ValueError,
        yaml.YAMLError,
    ) as exception:
        temporary_node = Node(
            "named_position_commander_error"
        )

        temporary_node.get_logger().error(str(exception))
        temporary_node.destroy_node()

        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
