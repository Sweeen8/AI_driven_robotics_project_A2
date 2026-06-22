cmake_minimum_required(VERSION 3.8)

project(a2_workcell_description)

if(CMAKE_COMPILER_IS_GNUCXX OR
   CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(
  add_workcell_collision_objects
  src/add_workcell_collision_objects.cpp
)

ament_target_dependencies(
  add_workcell_collision_objects
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
)

install(
  TARGETS
    add_workcell_collision_objects
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/robot_position_monitor.py
    scripts/named_position_commander.py
    scripts/RobotController.py
    scripts/functions.py
    scripts/cartesian_sort_arch_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
    rviz
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


#Height of blocker 37,5 cm en Breedte is 30 cm. dikte is 0.5 mm standard is 10 cm  en 5 cm.
