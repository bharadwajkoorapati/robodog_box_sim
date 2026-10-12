cmake_minimum_required(VERSION 3.8)
project(ultrasonic_box_bot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(DIRECTORY
  urdf
  launch
  rviz
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)


add_executable(ultrasonic_listener src/ultrasonic_listener.cpp)
add_executable(lidar_detector src/lidar_detector.cpp)
add_executable(target_cube_publisher src/target_cube_publisher.cpp)
add_executable(target_cube_listener src/target_cube_listener.cpp)
add_executable(autonavigator_node src/autonavigator_node.cpp)

ament_target_dependencies(lidar_detector rclcpp sensor_msgs)
ament_target_dependencies(ultrasonic_listener rclcpp sensor_msgs)
ament_target_dependencies(target_cube_publisher rclcpp geometry_msgs)
ament_target_dependencies(target_cube_listener rclcpp geometry_msgs)
ament_target_dependencies(autonavigator_node rclcpp geometry_msgs nav_msgs sensor_msgs)

install(TARGETS
  ultrasonic_listener
  lidar_detector
  target_cube_publisher
  target_cube_listener
  autonavigator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

