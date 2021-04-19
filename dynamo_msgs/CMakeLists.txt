cmake_minimum_required(VERSION 3.0.2)
project(dynamo_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  actionlib_msgs
)

add_action_files(
  FILES
  walk_action.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
