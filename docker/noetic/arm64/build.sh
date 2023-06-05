#!/bin/bash
set -euo pipefail

# GH
IFM3D_CLONE_REPO=https://github.com/ifm/ifm3d
IFM3D_ROS_CLONE_REPO=https://github.com/ifm/ifm3d-ros
docker build -t ifm3d-ros:noetic-arm64v8 --build-arg IFM3D_CLONE_REPO=$IFM3D_CLONE_REPO --build-arg IFM3D_ROS_CLONE_REPO=$IFM3D_ROS_CLONE_REPO -f Dockerfile .


# GL
# user_name=$1
# token=$2
# IFM3D_CLONE_REPO=https://$user_name:$token@gitlab.dev.ifm/syntron/support/csr/ifm3d/ifm3d
# IFM3D_ROS_CLONE_REPO=https://$user_name:$token@gitlab.dev.ifm/syntron/support/csr/ifm3d/ifm3d-ros
# docker build -t ifm3d-ros:noetic-arm64v8 --build-arg IFM3D_CLONE_REPO=$IFM3D_CLONE_REPO --build-arg IFM3D_ROS_CLONE_REPO=$IFM3D_ROS_CLONE_REPO -f Dockerfile .
