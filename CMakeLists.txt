cmake_minimum_required(VERSION 3.5)
project(tms_if_for_opera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# MoveIt2関連のターゲット
set(MOVEIT2_TARGETS
  moveit2_change_pose_action_server
  moveit2_change_pose_plan_action_server
  moveit2_excavate_simple_action_server
  moveit2_excavate_simple_plan_action_server
  moveit2_release_simple_action_server
  scene_manager
)

# Navigation2関連のターゲット
set(NAVIGATION2_TARGETS
  navigation2_navigate_through_poses
)

# その他のターゲット
set(OTHERS_TARGETS

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

add_library(excavator_ik SHARED src/excavator_ik.cpp)
ament_target_dependencies(excavator_ik rclcpp urdf)

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
  excavator_ik
)

foreach(TARGET ${MOVEIT2_TARGETS})
  add_executable(${TARGET} src/moveit2/${TARGET}.cpp)
  ament_target_dependencies(${TARGET} ${COMMON_DEPENDENCIES})
  target_include_directories(${TARGET} PRIVATE ${COMMON_INCLUDE_DIRS})
  target_link_directories(${TARGET} PUBLIC ${COMMON_LINK_DIRS})
  target_link_libraries(${TARGET} ${COMMON_LINK_LIBS})
  install(TARGETS ${TARGET} DESTINATION lib/${PROJECT_NAME})
endforeach()

foreach(TARGET ${NAVIGATION2_TARGETS})
  add_executable(${TARGET} src/navigation2/${TARGET}.cpp)
  ament_target_dependencies(${TARGET} ${COMMON_DEPENDENCIES})
  target_include_directories(${TARGET} PRIVATE ${COMMON_INCLUDE_DIRS})
  target_link_directories(${TARGET} PUBLIC ${COMMON_LINK_DIRS})
  target_link_libraries(${TARGET} ${COMMON_LINK_LIBS})
  install(TARGETS ${TARGET} DESTINATION lib/${PROJECT_NAME})
endforeach()

foreach(TARGET ${OTHERS_TARGETS})
  add_executable(${TARGET} src/others/${TARGET}.cpp)
  ament_target_dependencies(${TARGET} ${COMMON_DEPENDENCIES})
  target_include_directories(${TARGET} PRIVATE ${COMMON_INCLUDE_DIRS})
  target_link_directories(${TARGET} PUBLIC ${COMMON_LINK_DIRS})
  target_link_libraries(${TARGET} ${COMMON_LINK_LIBS})
  install(TARGETS ${TARGET} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(TARGETS excavator_ik
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  collision_objects
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
