#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using namespace std::chrono_literals;

namespace
{

moveit_msgs::msg::CollisionObject make_box(
  const std::string & id,
  const std::string & frame_id,
  const double size_x,
  const double size_y,
  const double size_z,
  const double x,
  const double y,
  const double z,
  const double yaw = 0.0)
{
  moveit_msgs::msg::CollisionObject object;

  object.header.frame_id = frame_id;
  object.id = id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions.resize(3);

  primitive.dimensions[
    shape_msgs::msg::SolidPrimitive::BOX_X
  ] = size_x;

  primitive.dimensions[
    shape_msgs::msg::SolidPrimitive::BOX_Y
  ] = size_y;

  primitive.dimensions[
    shape_msgs::msg::SolidPrimitive::BOX_Z
  ] = size_z;

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  // Quaternion for rotation around the Z-axis.
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(yaw / 2.0);
  pose.orientation.w = std::cos(yaw / 2.0);

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation =
    moveit_msgs::msg::CollisionObject::ADD;

  return object;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "add_workcell_collision_objects"
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  moveit::planning_interface::PlanningSceneInterface
    planning_scene_interface;

  RCLCPP_INFO(
    node->get_logger(),
    "Waiting for MoveIt Planning Scene..."
  );

  rclcpp::sleep_for(3s);

  std::vector<moveit_msgs::msg::CollisionObject> objects;

  const std::string frame = "environment_root";

  // ==========================================================
  // Main table
  // ==========================================================

  // Brown tabletop.
  objects.push_back(
    make_box(
      "table_top_collision",
      frame,
      1.60,
      0.80,
      0.025,
      0.0,
      0.0,
      0.7375
    )
  );

  // Four table legs.
  objects.push_back(
    make_box(
      "table_leg_front_left_collision",
      frame,
      0.032,
      0.032,
      0.725,
      0.684,
      -0.284,
      0.3625
    )
  );

  objects.push_back(
    make_box(
      "table_leg_front_right_collision",
      frame,
      0.032,
      0.032,
      0.725,
      -0.684,
      -0.284,
      0.3625
    )
  );

  objects.push_back(
    make_box(
      "table_leg_back_left_collision",
      frame,
      0.032,
      0.032,
      0.725,
      0.684,
      0.284,
      0.3625
    )
  );

  objects.push_back(
    make_box(
      "table_leg_back_right_collision",
      frame,
      0.032,
      0.032,
      0.725,
      -0.684,
      0.284,
      0.3625
    )
  );

  // ==========================================================
  // Raised robot work platform
  // ==========================================================

  objects.push_back(
    make_box(
      "workcell_platform_collision",
      frame,
      0.80,
      0.60,
      0.019,
      0.0,
      0.0,
      0.7995
    )
  );

  // Four 4-cm support feet.
  objects.push_back(
    make_box(
      "platform_support_front_left_collision",
      frame,
      0.060,
      0.060,
      0.040,
      0.34,
      -0.24,
      0.770
    )
  );

  objects.push_back(
    make_box(
      "platform_support_front_right_collision",
      frame,
      0.060,
      0.060,
      0.040,
      -0.34,
      -0.24,
      0.770
    )
  );

  objects.push_back(
    make_box(
      "platform_support_back_left_collision",
      frame,
      0.060,
      0.060,
      0.040,
      0.34,
      0.24,
      0.770
    )
  );

  objects.push_back(
    make_box(
      "platform_support_back_right_collision",
      frame,
      0.060,
      0.060,
      0.040,
      -0.34,
      0.24,
      0.770
    )
  );

  // ==========================================================
  // Robot mounting plate
  // ==========================================================

  /*
   * The physical plate is approximately:
   * 0.185 x 0.130 x 0.005 m
   *
   * It directly touches the robot base. Adding the full plate
   * may cause MoveIt to report that link_base starts in collision.
   *
   * Therefore, the plate collision geometry is split into two
   * narrow side strips, leaving the robot-base centre free.
   */

  objects.push_back(
    make_box(
      "mounting_plate_left_collision",
      frame,
      0.045,
      0.130,
      0.004,
      0.070,
      0.235,
      0.811
    )
  );

  objects.push_back(
    make_box(
      "mounting_plate_right_collision",
      frame,
      0.045,
      0.130,
      0.004,
      -0.070,
      0.235,
      0.811
    )
  );

  // ==========================================================
  // Chair
  // The chair is rotated 180 degrees around Z, but its square
  // seat and legs retain the same collision dimensions.
  // ==========================================================

  objects.push_back(
    make_box(
      "chair_seat_collision",
      frame,
      0.45,
      0.45,
      0.030,
      0.0,
      -0.80,
      0.465
    )
  );

  objects.push_back(
    make_box(
      "chair_backrest_collision",
      frame,
      0.45,
      0.025,
      0.45,
      0.0,
      -1.0125,
      0.705
    )
  );

  objects.push_back(
    make_box(
      "chair_leg_front_left_collision",
      frame,
      0.030,
      0.030,
      0.450,
      -0.185,
      -0.615,
      0.225
    )
  );

  objects.push_back(
    make_box(
      "chair_leg_front_right_collision",
      frame,
      0.030,
      0.030,
      0.450,
      0.185,
      -0.615,
      0.225
    )
  );

  objects.push_back(
    make_box(
      "chair_leg_back_left_collision",
      frame,
      0.030,
      0.030,
      0.450,
      -0.185,
      -0.985,
      0.225
    )
  );

  objects.push_back(
    make_box(
      "chair_leg_back_right_collision",
      frame,
      0.030,
      0.030,
      0.450,
      0.185,
      -0.985,
      0.225
    )
  );

  // ==========================================================
  // Blocker collision objects
  // ==========================================================
  //
  // These collision objects match the current URDF blocker assembly.
  //
  // URDF blocker setup:
  // - blocker assembly is attached to workcell_platform
  // - assembly origin: xyz="0.000 -0.130 0.000"
  // - assembly yaw: 1.5708 rad
  //
  // This file places collision objects in environment_root.
  // Therefore the local blocker z-values are added to the
  // workcell platform center height: 0.7995 m.

  const double blocker_yaw = 1.5708;
  const double platform_center_z = 0.7995;

  // Red blocker screen.
  objects.push_back(
    make_box(
      "blocker_screen_collision",
      frame,
      0.300,
      0.005,
      0.375,
      0.000,
      -0.130,
      platform_center_z + 0.272,
      blocker_yaw
    )
  );

  // Left standard/support.
  objects.push_back(
    make_box(
      "blocker_standard_left_collision",
      frame,
      0.080,
      0.050,
      0.100,
      0.000,
      -0.240,
      platform_center_z + 0.0595,
      blocker_yaw
    )
  );

  // Right standard/support.
  objects.push_back(
    make_box(
      "blocker_standard_right_collision",
      frame,
      0.080,
      0.050,
      0.100,
      0.000,
      -0.020,
      platform_center_z + 0.0595,
      blocker_yaw
    )
  );

  // ==========================================================
  // Apply collision objects
  // ==========================================================

  RCLCPP_INFO(
    node->get_logger(),
    "Applying %zu collision objects...",
    objects.size()
  );

  bool success = false;

  for (int attempt = 1; attempt <= 10; ++attempt) {
    success =
      planning_scene_interface.applyCollisionObjects(objects);

    if (success) {
      break;
    }

    RCLCPP_WARN(
      node->get_logger(),
      "Attempt %d failed; retrying...",
      attempt
    );

    rclcpp::sleep_for(1s);
  }

  if (!success) {
    RCLCPP_ERROR(
      node->get_logger(),
      "MoveIt did not accept the collision objects."
    );

    executor.cancel();

    if (spin_thread.joinable()) {
      spin_thread.join();
    }

    rclcpp::shutdown();
    return 1;
  }

  const auto known_objects =
    planning_scene_interface.getKnownObjectNames();

  RCLCPP_INFO(
    node->get_logger(),
    "Collision objects successfully added."
  );

  for (const auto & name : known_objects) {
    RCLCPP_INFO(
      node->get_logger(),
      "Known object: %s",
      name.c_str()
    );
  }

  executor.cancel();

  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}