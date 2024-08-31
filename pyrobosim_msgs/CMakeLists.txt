cmake_minimum_required(VERSION 3.22)
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

# Generate custom interfaces
set(msg_files
  "msg/ExecutionResult.msg"
  "msg/GoalPredicate.msg"
  "msg/GoalSpecification.msg"
  "msg/HallwayState.msg"
  "msg/LocationState.msg"
  "msg/ObjectState.msg"
  "msg/Path.msg"
  "msg/RobotState.msg"
  "msg/TaskAction.msg"
  "msg/TaskPlan.msg"
  "msg/WorldState.msg"
)

set(srv_files
  "srv/RequestWorldState.srv"
  "srv/SetLocationState.srv"
)

set(action_files
  "action/DetectObjects.action"
  "action/ExecuteTaskAction.action"
  "action/ExecuteTaskPlan.action"
  "action/FollowPath.action"
  "action/PlanPath.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES geometry_msgs
)

ament_package()
