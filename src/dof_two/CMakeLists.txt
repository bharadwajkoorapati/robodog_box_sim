cmake_minimum_required(VERSION 3.8)
project(dof_two)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
