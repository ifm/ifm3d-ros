#!/bin/sh

print_help()
{
  echo Convenience script for launching a ifm3d-ros launchfile from a docker container.
}

test $# -lt 1 && print_help


# IMAGE="nexus.ifm.com:20443/ifm-robotics/develop/ros1:amd64-ifm3d-ros-slim-visionfair"
IMAGE="ifm3d-ros:noetic-x86-64-v123"
LAUNCHFILE=camera_3d.launch

# Including "-it" so that CTRL-C works
docker run -it -p 11311:11311 $IMAGE \
    sh -c ". /opt/ros/noetic/setup.sh; \
    . /home/ifm/catkin_ws/ifm3d-ros/devel/setup.sh; \
    export ROS_MASTER_URI=http://172.17.0.2:11311; \
    export ROS_IP=172.17.0.2; \
    echo $ROS_MASTER_URI; \
    echo $ROS_IP; \
    roslaunch ifm3d_ros_driver $LAUNCHFILE pcic_port:=50010 timeout_millis:=1000 timeout_tolerance_secs:=10 $@"
