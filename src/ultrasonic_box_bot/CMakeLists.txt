cmake_minimum_required(VERSION 3.8)
project(ultrasonic_box_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  launch
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
