cmake_minimum_required(VERSION 3.5)
project(pyrobosim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enforce dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/TaskAction.msg"
  "msg/TaskPlan.msg"
  DEPENDENCIES geometry_msgs
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/test_command_publisher.py
  scripts/test_script.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install data files
install(DIRECTORY
  ${PROJECT_NAME}/data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()