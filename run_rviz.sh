#!/bin/sh

print_help()
{
  echo Convenience script for launching RVIZ for a ROS launchfile running a docker container. \ The default ROS_MASTER_URI is http://172.17.0.2:11311.
  exit 22
}

# source local ROS installation and setup link to ROS master
. /opt/ros/noetic/setup.sh
export ROS_MASTER_URI=http://172.17.0.2:11311

# run RVIZ
rosrun rviz rviz