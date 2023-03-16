#!/bin/bash
set -euo pipefail

# GH: arm64
IFM3D_CLONE_REPO=https://github.com/ifm/ifm3d
IFM3D_ROS_CLONE_REPO=https://github.com/ifm/ifm3d-ros
BASE_IMAGE_TAG=noetic-perception
BASE_IMAGE=arm64v8/ros
TAG=ifm3d-ros:noetic-arm64v8

# docker build -t ifm3d-ros:noetic-arm64v8 --no-cache --rm --build-arg IFM3D_CLONE_REPO=$IFM3D_CLONE_REPO --build-arg IFM3D_ROS_CLONE_REPO=$IFM3D_ROS_CLONE_REPO  \
#     --build-arg BASE_IMAGE=ros --build-arg BASE_IMAGE_TAG=noetic-perception -f Dockerfile .

docker build -t $TAG --build-arg IFM3D_CLONE_REPO=$IFM3D_CLONE_REPO --build-arg IFM3D_ROS_CLONE_REPO=$IFM3D_ROS_CLONE_REPO  \
    --build-arg BASE_IMAGE=$BASE_IMAGE --build-arg BASE_IMAGE_TAG=$BASE_IMAGE_TAG -f Dockerfile .


# GL: arm64
# user_name=$1
# token=$2
# IFM3D_CLONE_REPO=https://$user_name:$token@gitlab.dev.ifm/syntron/support/csr/ifm3d/ifm3d
# IFM3D_ROS_CLONE_REPO=https://$user_name:$token@gitlab.dev.ifm/syntron/support/csr/ifm3d/ifm3d-ros
# BASE_IMAGE_TAG=noetic-perception
# BASE_IMAGE=arm64v8/ros
# TAG=ifm3d-ros:noetic-arm64v8

# docker build -t ifm3d-ros:noetic-arm64v8 --no-cache --rm --build-arg IFM3D_CLONE_REPO=$IFM3D_CLONE_REPO --build-arg IFM3D_ROS_CLONE_REPO=$IFM3D_ROS_CLONE_REPO  \
#     --build-arg BASE_IMAGE=ros --build-arg BASE_IMAGE_TAG=noetic-perception -f Dockerfile .

# docker build -t $TAG --build-arg IFM3D_CLONE_REPO=$IFM3D_CLONE_REPO --build-arg IFM3D_ROS_CLONE_REPO=$IFM3D_ROS_CLONE_REPO  \
#     --build-arg BASE_IMAGE=$BASE_IMAGE --build-arg BASE_IMAGE_TAG=$BASE_IMAGE_TAG -f Dockerfile .