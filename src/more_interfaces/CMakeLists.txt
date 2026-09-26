cmake_minimum_required(VERSION 3.8)
project(more_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find basic dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define custom interface message files
set(msg_files
  "msg/AddressBook.msg"
)

# Generate the custom message targets
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

# Set up dependencies for packages that consume this interface package
ament_export_dependencies(rosidl_default_runtime)

# Define Executables
add_executable(publish_address_book src/publish_address_book.cpp)
add_executable(subscribe_address_book src/subscribe_address_book.cpp)

# --- CRITICAL FIX: Bind targets to the package interface generation layer ---
rosidl_target_interfaces(publish_address_book ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(subscribe_address_book ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Map target dependencies to core ROS 2 libraries
ament_target_dependencies(publish_address_book rclcpp)
ament_target_dependencies(subscribe_address_book rclcpp)

# Link both executables to your generated message headers and libraries
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(publish_address_book "${cpp_typesupport_target}")
target_link_libraries(subscribe_address_book "${cpp_typesupport_target}")

# Install targets so they can be run using 'ros2 run'
install(TARGETS
    publish_address_book 
    subscribe_address_book
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
