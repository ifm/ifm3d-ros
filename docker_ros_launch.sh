#!/bin/sh

print_help()
{
  echo Convenience script for launching a ifm3d-ros2 launchfile from a docker container.
}

test $# -lt 1 && print_help

IMAGE="ifm3d-ros:noetic-x86-64-v123"

docker run -it --network host $IMAGE \
    sh -c ". /opt/ros/noetic/setup.sh; \
    . /home/ifm/catkin_ws/devel/setup.sh; \
    roslaunch ifm3d_ros_driver camera_3d.launch  $@"
