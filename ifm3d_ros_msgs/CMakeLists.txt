cmake_minimum_required(VERSION 3.0.2)
project(ifm3d_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  tf2_ros
)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  DIRECTORY msg
  FILES
  Extrinsics.msg
  )

add_service_files(
  DIRECTORY srv
  FILES
  Dump.srv
  Config.srv
  Trigger.srv
  SoftOff.srv
  SoftOn.srv
  SyncClocks.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )


###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS message_runtime std_msgs
)


#############
## Install ##
#############

catkin_install_python(
  PROGRAMS
  bin/dump
  bin/config
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
