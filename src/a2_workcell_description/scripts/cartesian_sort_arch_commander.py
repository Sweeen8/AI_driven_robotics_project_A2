#!/usr/bin/env python3

"""
Cartesian arch movement script for the Lite 6 sorting task.

This script moves between two predefined Cartesian positions:

1. Pick position:
   Right side of blocker / object detection side.

2. Sort position:
   Left side of blocker / drop side.

The movement is planned as an arch:
current side -> high point above blocker -> target side

Default mode is plan-only.
Use execute:=true only after the path is visually safe in RViz.
"""

import math
from typing import List, Tuple
import time
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import DisplayTrajectory, RobotState
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


# ==========================================================
# Helper math
# ==========================================================

def degrees_to_radians(value: float) -> float:
    return value * math.pi / 180.0


def quaternion_from_rpy(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> Quaternion:
    """
    Convert roll, pitch, yaw in degrees to a quaternion.
    """

    roll = degrees_to_radians(roll_deg)
    pitch = degrees_to_radians(pitch_deg)
    yaw = degrees_to_radians(yaw_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    quaternion = Quaternion()
    quaternion.w = (cr * cp * cy) + (sr * sp * sy)
    quaternion.x = (sr * cp * cy) - (cr * sp * sy)
    quaternion.y = (cr * sp * cy) + (sr * cp * sy)
    quaternion.z = (cr * cp * sy) - (sr * sp * cy)

    return quaternion


def normalize_quaternion(q: Quaternion) -> Quaternion:
    length = math.sqrt(
        (q.x * q.x) +
        (q.y * q.y) +
        (q.z * q.z) +
        (q.w * q.w)
    )

    if length == 0.0:
        q.w = 1.0
        return q

    result = Quaternion()
    result.x = q.x / length
    result.y = q.y / length
    result.z = q.z / length
    result.w = q.w / length

    return result


def slerp_quaternion(
    q1: Quaternion,
    q2: Quaternion,
    t: float,
) -> Quaternion:
    """
    Spherical linear interpolation between two quaternions.
    This gives smoother orientation changes during the arch.
    """

    q1 = normalize_quaternion(q1)
    q2 = normalize_quaternion(q2)

    dot = (
        (q1.x * q2.x) +
        (q1.y * q2.y) +
        (q1.z * q2.z) +
        (q1.w * q2.w)
    )

    if dot < 0.0:
        q2.x *= -1.0
        q2.y *= -1.0
        q2.z *= -1.0
        q2.w *= -1.0
        dot *= -1.0

    if dot > 0.9995:
        result = Quaternion()
        result.x = q1.x + (t * (q2.x - q1.x))
        result.y = q1.y + (t * (q2.y - q1.y))
        result.z = q1.z + (t * (q2.z - q1.z))
        result.w = q1.w + (t * (q2.w - q1.w))
        return normalize_quaternion(result)

    theta_0 = math.acos(dot)
    theta = theta_0 * t

    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)

    scale_1 = math.cos(theta) - (dot * sin_theta / sin_theta_0)
    scale_2 = sin_theta / sin_theta_0

    result = Quaternion()
    result.x = (scale_1 * q1.x) + (scale_2 * q2.x)
    result.y = (scale_1 * q1.y) + (scale_2 * q2.y)
    result.z = (scale_1 * q1.z) + (scale_2 * q2.z)
    result.w = (scale_1 * q1.w) + (scale_2 * q2.w)

    return normalize_quaternion(result)


# ==========================================================
# Main commander
# ==========================================================

class CartesianSortArchCommander(Node):
    """
    Plans and optionally executes a Cartesian arch over the blocker.
    """

    def __init__(self) -> None:
        super().__init__("cartesian_sort_arch_commander")

        self.declare_parameter("group_name", "lite6")
        self.declare_parameter("reference_frame", "environment_root")
        self.declare_parameter("end_effector_frame", "link_eef")

        self.declare_parameter("direction", "pick_to_sort")

        # Plan only by default.
        self.declare_parameter("execute", False)

        # Arch height.
        # Your pick/sort z positions are around 1.05 m.
        # The blocker is tall, so 1.35 m is a safe first test.
        self.declare_parameter("arch_z", 1.35)

        # Cartesian interpolation settings.
        self.declare_parameter("waypoint_count", 40)
        self.declare_parameter("max_step", 0.01)
        self.declare_parameter("jump_threshold", 0.0)
        self.declare_parameter("min_fraction", 0.90)
        self.declare_parameter("avoid_collisions", True)

        self.group_name = str(self.get_parameter("group_name").value)
        self.reference_frame = str(
            self.get_parameter("reference_frame").value
        )
        self.end_effector_frame = str(
            self.get_parameter("end_effector_frame").value
        )

        self.direction = str(self.get_parameter("direction").value)
        self.execute = bool(self.get_parameter("execute").value)
        self.arch_z = float(self.get_parameter("arch_z").value)

        self.waypoint_count = int(
            self.get_parameter("waypoint_count").value
        )
        self.max_step = float(self.get_parameter("max_step").value)
        self.jump_threshold = float(
            self.get_parameter("jump_threshold").value
        )
        self.min_fraction = float(
            self.get_parameter("min_fraction").value
        )
        self.avoid_collisions = bool(
            self.get_parameter("avoid_collisions").value
        )

        if self.waypoint_count < 10:
            self.waypoint_count = 10

        # Pick position: right side / object detection side.
        self.pick_position = {
            "x": 0.196543,
            "y": -0.081481,
            "z": 1.046044,
            "roll": -179.999,
            "pitch": 0.003,
            "yaw": -68.064,
        }

        # Sort/drop position: left side of blocker.
        self.sort_position = {
            "x": -0.182879,
            "y": -0.091808,
            "z": 1.078598,
            "roll": 179.994,
            "pitch": -0.004,
            "yaw": -89.991,
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.cartesian_client = self.create_client(
            GetCartesianPath,
            "/compute_cartesian_path",
        )

        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            "/execute_trajectory",
        )

        self.display_publisher = self.create_publisher(
            DisplayTrajectory,
            "/display_planned_path",
            10,
        )

        self.started = False
        self.timer = self.create_timer(
            0.5,
            self.start_once,
        )

        self.get_logger().info("Cartesian sort arch commander started.")
        self.get_logger().info(f"Direction: {self.direction}")
        self.get_logger().info(f"Execute: {self.execute}")
        self.get_logger().info(f"Arch height: {self.arch_z}")

    def start_once(self) -> None:
        if self.started:
            return

        if not self.cartesian_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info(
                "Waiting for /compute_cartesian_path service..."
            )
            return

        self.started = True
        self.timer.cancel()

        start_pose = self.get_current_pose()

        if start_pose is None:
            self.get_logger().error("Could not get current pose.")
            rclpy.shutdown()
            return

        target_pose = self.get_target_pose()

        if target_pose is None:
            self.get_logger().error(
                "Invalid direction. Use pick_to_sort or sort_to_pick."
            )
            rclpy.shutdown()
            return

        waypoints = self.create_arch_waypoints(
            start_pose=start_pose,
            target_pose=target_pose,
        )

        self.get_logger().info(
            f"Generated {len(waypoints)} arch waypoints."
        )

        self.request_cartesian_path(waypoints)

    def get_current_pose(self) -> PoseStamped | None:
        timeout_seconds = 10.0
        start_time = self.get_clock().now().nanoseconds / 1e9

        while True:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.reference_frame,
                    self.end_effector_frame,
                    rclpy.time.Time(),
                )

                pose = PoseStamped()
                pose.header = transform.header
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation

                self.get_logger().info(
                    "Current end-effector pose: "
                    f"x={pose.pose.position.x:.4f}, "
                    f"y={pose.pose.position.y:.4f}, "
                    f"z={pose.pose.position.z:.4f}"
                )

                return pose

            except TransformException as exception:
                current_time = self.get_clock().now().nanoseconds / 1e9

                if current_time - start_time > timeout_seconds:
                    self.get_logger().error(
                        f"Could not lookup transform "
                        f"{self.reference_frame} -> {self.end_effector_frame}: "
                        f"{exception}"
                    )
                    return None
                
                self.get_logger().info(
                    r"Waiting for TF"
                    f"{self.reference_frame} -> {self.end_effector_frame}"
                )

                time.sleep(0.5)

    def get_target_pose(self) -> PoseStamped | None:
        if self.direction == "pick_to_sort":
            target_data = self.sort_position
            target_name = "sort/drop"
        elif self.direction == "sort_to_pick":
            target_data = self.pick_position
            target_name = "pick"
        else:
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.reference_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = target_data["x"]
        pose.pose.position.y = target_data["y"]
        pose.pose.position.z = target_data["z"]

        pose.pose.orientation = quaternion_from_rpy(
            target_data["roll"],
            target_data["pitch"],
            target_data["yaw"],
        )

        self.get_logger().info(
            f"Target pose ({target_name}): "
            f"x={pose.pose.position.x:.4f}, "
            f"y={pose.pose.position.y:.4f}, "
            f"z={pose.pose.position.z:.4f}"
        )

        return pose

    def create_arch_waypoints(
        self,
        start_pose: PoseStamped,
        target_pose: PoseStamped,
    ) -> List[Pose]:
        waypoints: List[Pose] = []

        start_x = start_pose.pose.position.x
        start_y = start_pose.pose.position.y
        start_z = start_pose.pose.position.z

        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        target_z = target_pose.pose.position.z

        start_orientation = start_pose.pose.orientation
        target_orientation = target_pose.pose.orientation

        highest_endpoint = max(start_z, target_z)
        arch_extra_height = max(
            0.0,
            self.arch_z - highest_endpoint,
        )

        for index in range(1, self.waypoint_count + 1):
            t = index / self.waypoint_count

            pose = Pose()

            pose.position.x = start_x + ((target_x - start_x) * t)
            pose.position.y = start_y + ((target_y - start_y) * t)

            linear_z = start_z + ((target_z - start_z) * t)

            # Parabolic arch bump:
            # 0 at start, 1 at middle, 0 at end.
            arch_bump = 4.0 * t * (1.0 - t) * arch_extra_height

            pose.position.z = linear_z + arch_bump

            pose.orientation = slerp_quaternion(
                start_orientation,
                target_orientation,
                t,
            )

            waypoints.append(pose)

        return waypoints

    def request_cartesian_path(self, waypoints: List[Pose]) -> None:
        request = GetCartesianPath.Request()
        request.header.frame_id = self.reference_frame
        request.header.stamp = self.get_clock().now().to_msg()

        request.start_state = RobotState()
        request.start_state.is_diff = True

        request.group_name = self.group_name
        request.link_name = self.end_effector_frame
        request.waypoints = waypoints
        request.max_step = self.max_step
        request.jump_threshold = self.jump_threshold
        request.avoid_collisions = self.avoid_collisions

        self.get_logger().info("Requesting Cartesian path...")

        future = self.cartesian_client.call_async(request)
        future.add_done_callback(self.cartesian_response_callback)

    def cartesian_response_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as exception:
            self.get_logger().error(
                f"Cartesian path request failed: {exception}"
            )
            rclpy.shutdown()
            return

        fraction = response.fraction
        error_code = response.error_code.val

        self.get_logger().info(
            f"Cartesian path fraction: {fraction:.3f}"
        )
        self.get_logger().info(
            f"MoveIt error code: {error_code}"
        )

        self.publish_display_trajectory(response)

        if fraction < self.min_fraction:
            self.get_logger().error(
                "Cartesian path fraction is too low. "
                "The path is incomplete and will not execute."
            )
            rclpy.shutdown()
            return

        if not self.execute:
            self.get_logger().info(
                "Plan-only mode. Trajectory published to RViz."
            )
            rclpy.shutdown()
            return

        self.execute_trajectory(response.solution)

    def publish_display_trajectory(self, response) -> None:
        display = DisplayTrajectory()
        display.trajectory_start = response.start_state
        display.trajectory.append(response.solution)

        self.display_publisher.publish(display)

        self.get_logger().info(
            "Published Cartesian trajectory to /display_planned_path."
        )

    def execute_trajectory(self, trajectory) -> None:
        if not self.execute_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "Action server /execute_trajectory is not available."
            )
            rclpy.shutdown()
            return

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            "Sending Cartesian trajectory to /execute_trajectory..."
        )

        future = self.execute_client.send_goal_async(goal)
        future.add_done_callback(self.execute_goal_response_callback)

    def execute_goal_response_callback(self, future) -> None:
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Execution goal rejected.")
            rclpy.shutdown()
            return

        self.get_logger().info("Execution goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.execute_result_callback)

    def execute_result_callback(self, future) -> None:
        result = future.result().result
        error_code = result.error_code.val

        if error_code == result.error_code.SUCCESS:
            self.get_logger().info(
                "Cartesian arch trajectory executed successfully."
            )
        else:
            self.get_logger().error(
                f"Cartesian arch execution failed. Error code: {error_code}"
            )

        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = CartesianSortArchCommander()

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
