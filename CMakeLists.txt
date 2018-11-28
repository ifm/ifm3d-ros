cmake_minimum_required(VERSION 3.5)
project(ifm3d)

find_package(ifm3d 0.11.2 CONFIG REQUIRED COMPONENTS
             camera
             framegrabber
             image
             )

find_package(catkin REQUIRED COMPONENTS
             rospy
             image_transport
             pcl_ros
             cv_bridge
             nodelet
             roscpp
             sensor_msgs
             std_msgs
             tf
             message_generation
             message_runtime
             rostest
             )

catkin_python_setup()
#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  Extrinsics.msg
  )

add_service_files(
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
  INCLUDE_DIRS include
  LIBRARIES ifm3d_ros
  CATKIN_DEPENDS roscpp nodelet
  )

#############
##  Build  ##
#############
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(ifm3d_ros src/camera_nodelet.cpp)
target_link_libraries(ifm3d_ros
  ${catkin_LIBRARIES}
  ifm3d::camera
  ifm3d::framegrabber
  ifm3d::image
  )
add_dependencies(ifm3d_ros ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

catkin_install_python(
  PROGRAMS
  bin/dump
  bin/config
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(TARGETS
  ifm3d_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

######################
## Node-level tests ##
######################

if (CATKIN_ENABLE_TESTING)
  add_rostest(test/ifm3d.test)
  catkin_add_nosetests(test)
endif()
