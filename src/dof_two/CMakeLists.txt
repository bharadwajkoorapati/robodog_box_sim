cmake_minimum_required(VERSION 3.8)

project(dof_two)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(
  keyboard_joint_controller
  src/keyboard_joint_controller.cpp
)

target_compile_features(keyboard_joint_controller PUBLIC cxx_std_17)

ament_target_dependencies(
  keyboard_joint_controller
  rclcpp
  sensor_msgs
  std_msgs
  trajectory_msgs
)

install(
  TARGETS
    keyboard_joint_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    urdf
    worlds
    meshes
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
