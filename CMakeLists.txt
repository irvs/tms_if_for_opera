cmake_minimum_required(VERSION 3.5)
project(tms_if_for_opera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(EXCAVATOR_TARGETS
  tms_if_moveit_action_server
  # excavator_assist_excavation_pose_to_joint_angles
  excavator_navigate_through_poses
  excavator_navigate_anywhere
  excavator_follow_waypoints
)

set(CRAWLERDUMP_TARGETS
  crawlerdump_navigate_through_poses
  crawlerdump_navigate_anywhere
  crawlerdump_follow_waypoints
  crawlerdump_release_soil
  crawlerdump_swing
  crawlerdump_swing_align_to_heading
)

set(BULLDOZER_TARGETS
  bulldozer_blade_control
  bulldozer_navigate_through_poses
  bulldozer_navigate_anywhere
  bulldozer_follow_waypoints
)

find_package(ament_cmake REQUIRED)
find_package(tms_msg_rp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(bsoncxx REQUIRED)
find_package(mongocxx REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(com3_msgs REQUIRED)

set(COMMON_DEPENDENCIES
  "rclcpp"
  "rclcpp_action"
  "moveit_common"
  "moveit_ros_planning_interface"
  "rosidl_default_runtime"
  "std_msgs"
  "tf2"
  "tf2_geometry_msgs"

  "nav2_msgs"
  "com3_msgs"
  
  "Boost"
  "mongocxx"
  "bsoncxx"

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

foreach(TARGET ${EXCAVATOR_TARGETS})
  add_executable(${TARGET} src/Excavator/${TARGET}.cpp)
  ament_target_dependencies(${TARGET} ${COMMON_DEPENDENCIES})
  target_include_directories(${TARGET} PRIVATE ${COMMON_INCLUDE_DIRS})
  target_link_directories(${TARGET} PUBLIC ${COMMON_LINK_DIRS})
  target_link_libraries(${TARGET} ${COMMON_LINK_LIBS})
  install(TARGETS ${TARGET} DESTINATION lib/${PROJECT_NAME})
endforeach()

foreach(TARGET ${CRAWLERDUMP_TARGETS})
  add_executable(${TARGET} src/Crawlerdump/${TARGET}.cpp)
  ament_target_dependencies(${TARGET} ${COMMON_DEPENDENCIES})
  target_include_directories(${TARGET} PRIVATE ${COMMON_INCLUDE_DIRS})
  target_link_directories(${TARGET} PUBLIC ${COMMON_LINK_DIRS})
  target_link_libraries(${TARGET} ${COMMON_LINK_LIBS})
  install(TARGETS ${TARGET} DESTINATION lib/${PROJECT_NAME})
endforeach()

foreach(TARGET ${BULLDOZER_TARGETS})
  add_executable(${TARGET} src/Bulldozer/${TARGET}.cpp)
  ament_target_dependencies(${TARGET} ${COMMON_DEPENDENCIES})
  target_include_directories(${TARGET} PRIVATE ${COMMON_INCLUDE_DIRS})
  target_link_directories(${TARGET} PUBLIC ${COMMON_LINK_DIRS})
  target_link_libraries(${TARGET} ${COMMON_LINK_LIBS})
  install(TARGETS ${TARGET} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY
  launch
  collision_objects
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
