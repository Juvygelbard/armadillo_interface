cmake_minimum_required(VERSION 2.8.3)
project(cpp_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  move_base_msgs
  actionlib
  tf
  costmap_2d
  base_local_planner

  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(drive
  src/robot_fsm.cpp
  include/cpp_robot/robot_fsm.h
  src/head_interface.cpp
  include/cpp_robot/head_interface.h
  src/arm_interface.cpp
  src/object_handler.cpp
  src/driver_interface.cpp
  src/drive.cpp)
target_link_libraries(drive ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(arm
  src/arm.cpp
)
add_dependencies(arm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm
  ${catkin_LIBRARIES}
)