cmake_minimum_required(VERSION 3.5)
project(tms_if_for_opera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(TARGETS
  # zx200_trajectory_execution_action_server
  zx200_change_pose_action_server
  # zx200_excavate_simple_action_server
  zx200_change_pose_action_client
)

find_package(ament_cmake REQUIRED)
find_package(tms_msg_rp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# find_package(shape_msgs REQUIRED)

find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(bsoncxx REQUIRED)
find_package(mongocxx REQUIRED)

find_package(excavator_ik REQUIRED)

set(COMMON_DEPENDENCIES
  "rclcpp"
  "rclcpp_action"
  "moveit_common"
  "moveit_ros_planning_interface"
  "rosidl_default_runtime"
  # "shape_msgs"
  "tf2"
  "tf2_geometry_msgs"
  
  "Boost"
  "mongocxx"
  "bsoncxx"

  "excavator_ik"
  "tms_msg_rp"
)

set(COMMON_INCLUDE_DIRS
  include
  /usr/local/include/mongocxx/v_noabi
  /usr/local/include/bsoncxx/v_noabi
)

set(COMMON_LINK_DIRS
  /usr/local/lib
)

set(COMMON_LINK_LIBS
  mongocxx
  bsoncxx
)

foreach(TARGET ${TARGETS})
  add_executable(${TARGET} src/${TARGET}.cpp)
  ament_target_dependencies(${TARGET} ${COMMON_DEPENDENCIES})
  target_include_directories(${TARGET} PRIVATE ${COMMON_INCLUDE_DIRS})
  target_link_directories(${TARGET} PUBLIC ${COMMON_LINK_DIRS})
  target_link_libraries(${TARGET} ${COMMON_LINK_LIBS})
  install(TARGETS ${TARGET} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(test_zx200_change_pose_action_server test/test_zx200_change_pose_action_server.cpp)
#   ament_target_dependencies(test_zx200_change_pose_action_server ${COMMON_DEPENDENCIES})
#   target_include_directories(test_zx200_change_pose_action_server PRIVATE ${COMMON_INCLUDE_DIRS})
#   target_link_directories(test_zx200_change_pose_action_server PUBLIC ${COMMON_LINK_DIRS})
#   target_link_libraries(test_zx200_change_pose_action_server ${COMMON_LINK_LIBS})
# endif()

ament_package()
