cmake_minimum_required(VERSION 2.8.3)
project(cpp_robot)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
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

include_directories(include ${catkin_INCLUDE_DIRS})

add_action_files(
  DIRECTORY action
  FILES SimpleDriver.action
)

add_message_files()

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib_msgs
)

add_executable(drive
  src/robot_fsm.cpp
  src/head_interface.cpp
  src/arm_interface.cpp
  src/object_handler.cpp
  src/driver_interface.cpp
  src/drive.cpp)
target_link_libraries(drive ${catkin_LIBRARIES})