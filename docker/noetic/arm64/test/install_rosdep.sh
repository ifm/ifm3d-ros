#!/bin/bash

if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi

# install required ros dependencies
cd /home/ifm/catkin_ws/

apt-get install -y --no-install-recommends python-rosdep \
    python-rosinstall-generator \
    python-vcstool \
    python-rosinstall

# rosdep install --from-paths /home/ifm/catkin/src/ --ignore-src -r -y
