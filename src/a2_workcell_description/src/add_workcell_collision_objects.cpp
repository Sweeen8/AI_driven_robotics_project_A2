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

moveit_msgs::msg::CollisionObject make_cylinder(
  const std::string & id,
  const std::string & frame_id,
  const double radius,
  const double height,
  const double x,
  const double y,
  const double z)
{
  moveit_msgs::msg::CollisionObject object;

  object.header.frame_id = frame_id;
  object.id = id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  primitive.dimensions.resize(2);

  primitive.dimensions[
    shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT
  ] = height;

  primitive.dimensions[
    shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS
  ] = radius;

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;

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

  // Red blocker screen: 28 cm wide x 0.1 cm thick x 36.5 cm tall.
  // Hovers 1.9 cm above platform. Z centre = platform_center_z + 0.2110 m.
  objects.push_back(
    make_box(
      "blocker_screen_collision",
      frame,
      0.280,
      0.001,
      0.365,
      0.000,
      -0.130,
      platform_center_z + 0.2110,
      blocker_yaw
    )
  );

  // Left standard: 11 cm long x 2 cm thick x 6 cm tall.
  // Assembly local X=-0.0985 → world Y = -0.130 + (-0.0985) = -0.2285 m
  objects.push_back(
    make_box(
      "blocker_standard_left_collision",
      frame,
      0.110,
      0.020,
      0.060,
      0.000,
      -0.2285,
      platform_center_z + 0.0395,
      blocker_yaw
    )
  );

  // Right standard: 11 cm long x 2 cm thick x 6 cm tall.
  // Assembly local X=+0.0985 → world Y = -0.130 + 0.0985 = -0.0315 m
  objects.push_back(
    make_box(
      "blocker_standard_right_collision",
      frame,
      0.110,
      0.020,
      0.060,
      0.000,
      -0.0315,
      platform_center_z + 0.0395,
      blocker_yaw
    )
  );

  // ==========================================================
  // Camera standaard (vernieuwd — 2×2 cm vierkante profielen)
  // ==========================================================
  //
  // Positie: 5.5 cm van rechterrand (X=0.40) en 4 cm van achterrand (Y=0.30).
  //   paal X = 0.40 - 0.055 = 0.345 m
  //   paal Y = 0.30 - 0.040 = 0.260 m
  //   platform top = 0.7995 + 0.019/2 = 0.809 m
  //
  // Staande paal: 2×2×67.7 cm
  //   centrum Z = 0.809 + 0.677/2 = 1.1475 m
  //
  // Zijdelingse balk: 43 cm, yaw = +65°, locaal centrum x = -0.189 m, z = +0.3085 m
  //   X = 0.345 - 0.189*cos(65°) = 0.2651 m
  //   Y = 0.260 - 0.189*sin(65°) = 0.0887 m
  //   Z = 1.1475 + 0.3085         = 1.4560 m
  //
  // Camera: lokaal x = -0.343 m, z = 0.2760 m boven paalcentrum
  //   X = 0.345 - 0.343*cos(65°) = 0.2000 m
  //   Y = 0.260 - 0.343*sin(65°) = -0.0509 m
  //   Z = 1.1475 + 0.2760         = 1.4235 m
  {
    const double camera_bar_yaw = 65.0 * M_PI / 180.0;

    // Staande paal (2×2×67.7 cm vierkant profiel).
    objects.push_back(
      make_box(
        "camera_pole_collision",
        frame,
        0.020,
        0.020,
        0.677,
        0.345,
        0.260,
        1.1475
      )
    );

    // Zijdelingse balk (2×2×43 cm), gedraaid +45°.
    objects.push_back(
      make_box(
        "camera_bar_collision",
        frame,
        0.430,
        0.020,
        0.020,
        0.2651,
        0.0887,
        1.4560,
        camera_bar_yaw
      )
    );

    // Camera body (5×11×4.5 cm), gedraaid +65°.
    objects.push_back(
      make_box(
        "camera_body_collision",
        frame,
        0.050,
        0.110,
        0.045,
        0.2000,
        -0.0509,
        1.4235,
        camera_bar_yaw
      )
    );
  }

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