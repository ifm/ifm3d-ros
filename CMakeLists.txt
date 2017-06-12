cmake_minimum_required(VERSION 2.8.3)
project(ifm3d)

#
# ifm3d is not yet setup for `find_package` discovery,
# so for now we do it "the old way"
#
find_library(LIB_ifm3d_camera NAMES ifm3d_camera)
find_library(LIB_ifm3d_framegrabber NAMES ifm3d_framegrabber)
find_library(LIB_ifm3d_image NAMES ifm3d_image)
find_package(catkin REQUIRED COMPONENTS
             image_transport
             pcl_ros
             cv_bridge
             roscpp
             sensor_msgs
             std_msgs
             tf
             message_generation
             message_runtime
             )

#######################################
## Declare ROS messages and services ##
#######################################

# Coming soon, for now, just streaming points clouds
# in free-run mode

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
##  Build  ##
#############
add_compile_options(-std=c++14)

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(ifm3d_node src/ifm3d_node.cpp)
target_link_libraries(ifm3d_node
  ${catkin_LIBRARIES}
  ${LIB_ifm3d_camera}
  ${LIB_ifm3d_framegrabber}
  ${LIB_ifm3d_image}
  )
#add_dependencies(ifm3d_node ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

file(GLOB ifm3d_launch_files launch/*.launch)
install(FILES
  ${ifm3d_launch_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

file(GLOB ifm3d_rviz_files launch/*.rviz)
install(FILES
  ${ifm3d_rviz_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(TARGETS
  ifm3d_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

######################
## Node-level tests ##
######################

# Yes, I know... coming soon!
