cmake_minimum_required(VERSION 3.0.2)
project(ifm3d_ros_examples)

find_package(catkin REQUIRED COMPONENTS
  tf2_ros
  nodelet
  ifm3d_ros_driver
  ifm3d_ros_msgs
)


catkin_package()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)
