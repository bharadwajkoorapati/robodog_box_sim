cmake_minimum_required(VERSION 3.8)
project(ultrasonic_box_bot)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

install(DIRECTORY
  urdf
  launch
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)


add_executable(ultrasonic_listener src/ultrasonic_listener.cpp)
ament_target_dependencies(ultrasonic_listener rclcpp sensor_msgs)

install(TARGETS
  ultrasonic_listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
