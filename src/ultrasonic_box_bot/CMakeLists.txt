cmake_minimum_required(VERSION 3.8)
project(ultrasonic_box_bot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

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
add_executable(sensor_ready src/sensor_ready.cpp)
add_executable(navigation_ready src/navigation_ready.cpp)
add_executable(target_nav2_client src/target_nav2_client.cpp)
add_executable(cmd_vel_watchdog src/cmd_vel_watchdog.cpp)
add_executable(map_padder src/map_padder.cpp)
add_executable(nav_goal_sender src/nav_goal_sender.cpp)

ament_target_dependencies(lidar_detector rclcpp sensor_msgs)
ament_target_dependencies(ultrasonic_listener rclcpp sensor_msgs)
ament_target_dependencies(target_cube_publisher rclcpp geometry_msgs)
ament_target_dependencies(target_cube_listener rclcpp geometry_msgs)
ament_target_dependencies(autonavigator_node rclcpp geometry_msgs nav_msgs sensor_msgs)
ament_target_dependencies(sensor_ready rclcpp nav_msgs sensor_msgs tf2_ros)
ament_target_dependencies(navigation_ready rclcpp nav_msgs tf2_ros)
ament_target_dependencies(
  target_nav2_client
  rclcpp rclcpp_action nav2_msgs geometry_msgs tf2_ros tf2_geometry_msgs)
ament_target_dependencies(cmd_vel_watchdog rclcpp geometry_msgs)
ament_target_dependencies(map_padder rclcpp nav_msgs)
ament_target_dependencies(nav_goal_sender rclcpp rclcpp_action nav2_msgs geometry_msgs)

install(TARGETS
  ultrasonic_listener
  lidar_detector
  target_cube_publisher
  target_cube_listener
  autonavigator_node
  sensor_ready
  navigation_ready
  target_nav2_client
  cmd_vel_watchdog
  map_padder
  nav_goal_sender
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(cmd_vel_watchdog test/test_cmd_vel_watchdog.py TIMEOUT 60)
  ament_add_pytest_test(target_nav2_client test/test_target_nav2_client.py TIMEOUT 120)
  ament_add_pytest_test(autonavigator test/test_autonavigator.py TIMEOUT 120)
  ament_add_pytest_test(map_padder test/test_map_padder.py TIMEOUT 60)
  ament_add_pytest_test(nav_goal_sender test/test_nav_goal_sender.py TIMEOUT 120)
endif()

ament_package()

