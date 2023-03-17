#!/bin/bash
set -euo pipefail

# GH: amd64
IFM3D_CLONE_REPO=https://github.com/ifm/ifm3d
IFM3D_ROS_CLONE_REPO=https://github.com/ifm/ifm3d-ros
BASE_IMAGE_TAG=noetic-perception
BASE_IMAGE=ros
TAG=ifm3d-ros:noetic-x86_64_new

# docker build -t $TAG --no-cache --rm --build-arg IFM3D_CLONE_REPO=$IFM3D_CLONE_REPO --build-arg IFM3D_ROS_CLONE_REPO=$IFM3D_ROS_CLONE_REPO  \
    # --build-arg BASE_IMAGE=$BASE_IMAGE --build-arg BASE_IMAGE_TAG=$BASE_IMAGE_TAG -f Dockerfile .
docker build -t $TAG --build-arg IFM3D_CLONE_REPO=$IFM3D_CLONE_REPO --build-arg IFM3D_ROS_CLONE_REPO=$IFM3D_ROS_CLONE_REPO  \
    --build-arg BASE_IMAGE=$BASE_IMAGE --build-arg BASE_IMAGE_TAG=$BASE_IMAGE_TAG -f Dockerfile .
