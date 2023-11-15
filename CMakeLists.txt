cmake_minimum_required(VERSION 3.5)
project(tms_if_for_opera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(tms_msg_ts REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(excavator_ik REQUIRED)

add_executable(backhoe_trajectory_execution_action_server src/backhoe_trajectory_execution_action_server.cpp)
ament_target_dependencies(backhoe_trajectory_execution_action_server
  "rclcpp"
  "rclcpp_action"
  "tms_msg_ts"
  "moveit_common"
  "moveit_ros_planning_interface"
  "rosidl_default_runtime"
  "Boost")
target_include_directories(backhoe_trajectory_execution_action_server PRIVATE include)

add_executable(backhoe_change_pose_action_server src/backhoe_change_pose_action_server.cpp)
ament_target_dependencies(backhoe_change_pose_action_server
  "rclcpp"
  "rclcpp_action"
  "tms_msg_ts"
  "moveit_common"
  "moveit_ros_planning_interface"
  "rosidl_default_runtime"
  "Boost"
  "excavator_ik")
target_include_directories(backhoe_change_pose_action_server PRIVATE include)

add_executable(backhoe_trajectory_execution_action_client src/backhoe_trajectory_execution_action_client.cpp)
ament_target_dependencies(backhoe_trajectory_execution_action_client
  "rclcpp"
  "rclcpp_action"
  "tms_msg_ts"
)
target_include_directories(backhoe_trajectory_execution_action_client PRIVATE include)


install(TARGETS
  backhoe_trajectory_execution_action_server
  backhoe_change_pose_action_server
  backhoe_trajectory_execution_action_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
