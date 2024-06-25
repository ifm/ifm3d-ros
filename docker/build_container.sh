#!/bin/bash
set -euo pipefail

##############
# For AMD64:
##############
ARCH="amd64"
BASE_IMAGE="amd64/ros"

##############
# For ARM64V8:
##############
# ARCH="arm64v8"
# BASE_IMAGE="arm64v8/ros"
# TAG=ifm3d-ros_133:noetic-arm64_v8_1.1.2

##############
# Arguments common for both architecture
##############
BUILD_IMAGE_TAG="noetic"
FINAL_IMAGE_TAG="noetic-ros-core"
IFM3D_VERSION="1.5.3"
IFM3D_ROS_REPO="https://github.com/ifm/ifm3d-ros"
IFM3D_ROS_BRANCH="v1.1.2"
UBUNTU_VERSION="20.04"

TAG=ifm3d-ros_${IFM3D_VERSION}:${BUILD_IMAGE_TAG}-${ARCH}_${IFM3D_ROS_BRANCH}
##############
# Build the Docker container
##############
docker build -t $TAG \
    --build-arg ARCH=${ARCH} \
    --build-arg BASE_IMAGE=${BASE_IMAGE} \
    --build-arg BUILD_IMAGE_TAG=${BUILD_IMAGE_TAG} \
    --build-arg FINAL_IMAGE_TAG=${FINAL_IMAGE_TAG} \
    --build-arg IFM3D_VERSION=${IFM3D_VERSION} \
    --build-arg IFM3D_ROS_REPO=${IFM3D_ROS_REPO} \
    --build-arg IFM3D_ROS_BRANCH=${IFM3D_ROS_BRANCH} \
    --build-arg UBUNTU_VERSION=${UBUNTU_VERSION} \
    -f Dockerfile_buildstage_only .
