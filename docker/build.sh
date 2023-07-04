#!/bin/bash
set -euo pipefail

IFM3D_ROS_BRANCH="0.7.1"
IFM3D_TAG="0.20.3"

## Melodic
# BASE_IMAGE=amd64/ros:melodic-perception # amd64
BASE_IMAGE=arm64v8/ros:melodic-perception # arm64
ROS_DISTRO=melodic
LSB_RELEASE=bionic

## Noetic
# BASE_IMAGE=ros:noetic-perception # amd64
# # BASE_IMAGE=arm64v8/ros:noetic-perception # arm64
# ROS_DISTRO=noetic
# LSB_RELEASE=focal

docker build -t ifm3d-ros:melodic-arm64 \
    --build-arg BASE_IMAGE=$BASE_IMAGE \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    --build-arg LSB_RELEASE=$LSB_RELEASE \
    --build-arg IFM3D_TAG=$IFM3D_TAG \
    --build-arg IFM3D_ROS_BRANCH=$IFM3D_ROS_BRANCH -f Dockerfile .