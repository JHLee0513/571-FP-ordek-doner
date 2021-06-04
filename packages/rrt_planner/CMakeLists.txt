cmake_minimum_required(VERSION 2.8.3)
project(rrt_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs
  std_msgs
  sensor_msgs
)


include_directories(
# include
  environments
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()
catkin_package()
