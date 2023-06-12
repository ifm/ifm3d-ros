#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/ifm/catkin_ws/ifm3d-ros/devel/setup.bash
exec "$@"