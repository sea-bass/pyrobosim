cmake_minimum_required(VERSION 3.8)
project(pyrobosim_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enforce dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GoalPredicate.msg"
  "msg/GoalSpecification.msg"
  "msg/LocationState.msg"
  "msg/ObjectState.msg"
  "msg/Path.msg"
  "msg/RobotState.msg"
  "msg/TaskAction.msg"
  "msg/TaskPlan.msg"
  "msg/WorldState.msg"
  "srv/RequestWorldState.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
