cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs
  std_msgs
  sensor_msgs
  message_generation
)

include_directories(
# include
  msg
  files
  ${catkin_INCLUDE_DIRS}
)
catkin_python_setup()

add_message_files(
  FILES
  PredictedPose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package()

