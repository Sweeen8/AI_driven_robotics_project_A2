#!/usr/bin/env python3

"""Publish the current and most recently planned Lite 6 positions."""

import json
import math
from typing import Dict, List, Optional, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import DisplayTrajectory, RobotState
from moveit_msgs.srv import GetPositionFK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_rpy(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[float, float, float]:
    """Convert a quaternion to roll, pitch and yaw in radians."""

    sin_roll_cos_pitch = 2.0 * ((w * x) + (y * z))
    cos_roll_cos_pitch = 1.0 - (2.0 * ((x * x) + (y * y)))
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch = 2.0 * ((w * y) - (z * x))

    if abs(sin_pitch) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sin_pitch)
    else:
        pitch = math.asin(sin_pitch)

    sin_yaw_cos_pitch = 2.0 * ((w * z) + (x * y))
    cos_yaw_cos_pitch = 1.0 - (2.0 * ((y * y) + (z * z)))
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)

    return roll, pitch, yaw


def radians_to_degrees(values: Sequence[float]) -> List[float]:
    """Convert a sequence of angles from radians to degrees."""

    return [math.degrees(value) for value in values]


class RobotPositionMonitor(Node):
    """Monitor and publish current and planned Lite 6 states."""

    def __init__(self) -> None:
        super().__init__("robot_position_monitor")

        self.declare_parameter("reference_frame", "environment_root")
        self.declare_parameter("end_effector_frame", "link_eef")
        self.declare_parameter(
            "display_trajectory_topic",
            "/display_planned_path",
        )
        self.declare_parameter("fk_service", "/compute_fk")
        self.declare_parameter("publish_rate_hz", 2.0)

        self.reference_frame = str(
            self.get_parameter("reference_frame").value
        )
        self.end_effector_frame = str(
            self.get_parameter("end_effector_frame").value
        )
        self.display_trajectory_topic = str(
            self.get_parameter("display_trajectory_topic").value
        )
        self.fk_service_name = str(
            self.get_parameter("fk_service").value
        )
        publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )

        if publish_rate_hz <= 0.0:
            publish_rate_hz = 2.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.current_pose_publisher = self.create_publisher(
            PoseStamped,
            "/robot_monitor/current_pose",
            10,
        )

        self.current_joints_publisher = self.create_publisher(
            JointState,
            "/robot_monitor/current_joints",
            10,
        )

        self.planned_pose_publisher = self.create_publisher(
            PoseStamped,
            "/robot_monitor/planned_pose",
            10,
        )

        self.planned_joints_publisher = self.create_publisher(
            JointState,
            "/robot_monitor/planned_joints",
            10,
        )

        self.current_text_publisher = self.create_publisher(
            String,
            "/robot_monitor/current_state_text",
            10,
        )

        self.planned_text_publisher = self.create_publisher(
            String,
            "/robot_monitor/planned_state_text",
            10,
        )

        self.joint_state_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.trajectory_subscription = self.create_subscription(
            DisplayTrajectory,
            self.display_trajectory_topic,
            self.display_trajectory_callback,
            10,
        )

        self.fk_client = self.create_client(
            GetPositionFK,
            self.fk_service_name,
        )

        self.latest_joint_state: Optional[JointState] = None
        self.latest_current_pose: Optional[PoseStamped] = None

        self.timer = self.create_timer(
            1.0 / publish_rate_hz,
            self.publish_current_pose,
        )

        self.get_logger().info(
            "Robot position monitor started."
        )
        self.get_logger().info(
            f"Reference frame: {self.reference_frame}"
        )
        self.get_logger().info(
            f"End-effector frame: {self.end_effector_frame}"
        )
        self.get_logger().info(
            f"Planned trajectory topic: "
            f"{self.display_trajectory_topic}"
        )
        self.get_logger().info(
            f"Forward-kinematics service: "
            f"{self.fk_service_name}"
        )

    def joint_state_callback(self, message: JointState) -> None:
        """Store and republish the current robot joint state."""

        self.latest_joint_state = message
        self.current_joints_publisher.publish(message)

    def publish_current_pose(self) -> None:
        """Look up and publish the current end-effector pose."""

        try:
            transform = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.end_effector_frame,
                rclpy.time.Time(),
            )
        except TransformException as exception:
            self.get_logger().debug(
                f"Waiting for transform "
                f"{self.reference_frame} -> "
                f"{self.end_effector_frame}: {exception}"
            )
            return

        pose = PoseStamped()
        pose.header = transform.header

        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z

        pose.pose.orientation = transform.transform.rotation

        self.latest_current_pose = pose
        self.current_pose_publisher.publish(pose)

        self.publish_current_text(pose)

    def publish_current_text(self, pose: PoseStamped) -> None:
        """Publish a readable JSON summary of the current state."""

        quaternion = pose.pose.orientation

        roll, pitch, yaw = quaternion_to_rpy(
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )

        summary: Dict[str, object] = {
            "state": "current",
            "reference_frame": pose.header.frame_id,
            "end_effector_frame": self.end_effector_frame,
            "position_m": {
                "x": round(pose.pose.position.x, 6),
                "y": round(pose.pose.position.y, 6),
                "z": round(pose.pose.position.z, 6),
            },
            "orientation_deg": {
                "roll": round(math.degrees(roll), 3),
                "pitch": round(math.degrees(pitch), 3),
                "yaw": round(math.degrees(yaw), 3),
            },
        }

        if self.latest_joint_state is not None:
            joint_degrees = radians_to_degrees(
                self.latest_joint_state.position
            )

            summary["joint_positions_deg"] = {
                name: round(value, 3)
                for name, value in zip(
                    self.latest_joint_state.name,
                    joint_degrees,
                )
            }

        message = String()
        message.data = json.dumps(
            summary,
            indent=2,
            sort_keys=False,
        )

        self.current_text_publisher.publish(message)

    def display_trajectory_callback(
        self,
        message: DisplayTrajectory,
    ) -> None:
        """Read the final state from the most recent MoveIt plan."""

        if not message.trajectory:
            self.get_logger().warning(
                "Received a DisplayTrajectory without trajectories."
            )
            return

        robot_trajectory = message.trajectory[-1]
        joint_trajectory = robot_trajectory.joint_trajectory

        if not joint_trajectory.joint_names:
            self.get_logger().warning(
                "The planned trajectory has no joint names."
            )
            return

        if not joint_trajectory.points:
            self.get_logger().warning(
                "The planned trajectory has no trajectory points."
            )
            return

        final_point = joint_trajectory.points[-1]

        if len(final_point.positions) != len(
            joint_trajectory.joint_names
        ):
            self.get_logger().error(
                "Planned joint names and positions have "
                "different lengths."
            )
            return

        planned_joint_state = JointState()
        planned_joint_state.header.stamp = self.get_clock().now().to_msg()
        planned_joint_state.header.frame_id = self.reference_frame
        planned_joint_state.name = list(
            joint_trajectory.joint_names
        )
        planned_joint_state.position = list(
            final_point.positions
        )

        self.planned_joints_publisher.publish(
            planned_joint_state
        )

        self.request_planned_forward_kinematics(
            planned_joint_state
        )

    def request_planned_forward_kinematics(
        self,
        planned_joint_state: JointState,
    ) -> None:
        """Request the final end-effector pose for planned joints."""

        if not self.fk_client.service_is_ready():
            self.get_logger().warning(
                f"FK service {self.fk_service_name} is not ready. "
                "The planned joints were published, but the planned "
                "Cartesian pose could not be calculated."
            )

            self.publish_planned_text(
                planned_joint_state,
                None,
            )
            return

        request = GetPositionFK.Request()
        request.header.frame_id = self.reference_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.fk_link_names = [self.end_effector_frame]

        request.robot_state = RobotState()
        request.robot_state.joint_state = planned_joint_state

        future = self.fk_client.call_async(request)

        future.add_done_callback(
            lambda completed_future: self.fk_response_callback(
                completed_future,
                planned_joint_state,
            )
        )

    def fk_response_callback(
        self,
        future,
        planned_joint_state: JointState,
    ) -> None:
        """Publish the FK pose returned by MoveIt."""

        try:
            response = future.result()
        except Exception as exception:
            self.get_logger().error(
                f"FK request failed: {exception}"
            )
            return

        if response.error_code.val != response.error_code.SUCCESS:
            self.get_logger().error(
                "MoveIt could not calculate the planned "
                f"end-effector pose. Error code: "
                f"{response.error_code.val}"
            )

            self.publish_planned_text(
                planned_joint_state,
                None,
            )
            return

        if not response.pose_stamped:
            self.get_logger().error(
                "FK response did not contain a pose."
            )
            return

        planned_pose = response.pose_stamped[0]

        self.planned_pose_publisher.publish(planned_pose)

        self.publish_planned_text(
            planned_joint_state,
            planned_pose,
        )

    def publish_planned_text(
        self,
        joint_state: JointState,
        pose: Optional[PoseStamped],
    ) -> None:
        """Publish a readable JSON summary of the planned state."""

        joint_degrees = radians_to_degrees(
            joint_state.position
        )

        summary: Dict[str, object] = {
            "state": "planned_final",
            "reference_frame": self.reference_frame,
            "end_effector_frame": self.end_effector_frame,
            "joint_positions_deg": {
                name: round(value, 3)
                for name, value in zip(
                    joint_state.name,
                    joint_degrees,
                )
            },
        }

        if pose is not None:
            quaternion = pose.pose.orientation

            roll, pitch, yaw = quaternion_to_rpy(
                quaternion.x,
                quaternion.y,
                quaternion.z,
                quaternion.w,
            )

            summary["position_m"] = {
                "x": round(pose.pose.position.x, 6),
                "y": round(pose.pose.position.y, 6),
                "z": round(pose.pose.position.z, 6),
            }

            summary["orientation_deg"] = {
                "roll": round(math.degrees(roll), 3),
                "pitch": round(math.degrees(pitch), 3),
                "yaw": round(math.degrees(yaw), 3),
            }

        message = String()
        message.data = json.dumps(
            summary,
            indent=2,
            sort_keys=False,
        )

        self.planned_text_publisher.publish(message)

        self.get_logger().info(
            "Published the final state of the latest plan."
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = RobotPositionMonitor()

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