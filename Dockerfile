ARG ARCH="amd64"
ARG BASE_IMAGE="amd64/ros"
ARG BUILD_IMAGE_TAG="noetic"
ARG FINAL_IMAGE_TAG="noetic-ros-core"
ARG IFM3D_VERSION="1.2.6"
ARG IFM3D_ROS2_REPO="https://github.com/ifm/ifm3d-ros.git"
ARG IFM3D_ROS2_BRANCH="master"
ARG UBUNTU_VERSION="20.04"

FROM ${BASE_IMAGE}:${BUILD_IMAGE_TAG} AS build
ARG IFM3D_VERSION
ARG IFM3D_ROS_REPO
ARG IFM3D_ROS_BRANCH
ARG ARCH
ARG UBUNTU_VERSION

# Create the ifm user
RUN id ifm 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U ifm
WORKDIR /home/ifm

# Dependencies for both ifm3d and ifm3d-ros2
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    git \
    jq \
    libxmlrpc-c++8-dev \
    libproj-dev \
    build-essential \
    coreutils \
    cmake \
    wget \
    libssl-dev \
    libgoogle-glog-dev \
    libgoogle-glog0v5 \
    python3-rosdep

# Install ifm3d using the deb files
RUN mkdir /home/ifm/ifm3d
ADD https://github.com/ifm/ifm3d/releases/download/v${IFM3D_VERSION}/ifm3d-ubuntu-${UBUNTU_VERSION}-${ARCH}-debs_${IFM3D_VERSION}.tar /home/ifm/ifm3d
RUN cd /home/ifm/ifm3d &&\
    tar -xf ifm3d-ubuntu-${UBUNTU_VERSION}-${ARCH}-debs_${IFM3D_VERSION}.tar &&  \
    dpkg -i *.deb

# Clone and build ifm3d-ros repo
SHELL ["/bin/bash", "-c"]
# RUN mkdir -p /home/ifm/catkin_ws/src && \
#     cd /home/ifm/catkin_ws/src && \
    # git clone ${IFM3D_ROS_REPO} -b ${IFM3D_ROS_BRANCH} --single-branch 
ADD . /home/ifm/catkin_ws/src
RUN cd /home/ifm/catkin_ws && \
    rosdep update --rosdistro=${ROS_DISTRO} && \
    rosdep install --from-path src -y --ignore-src

RUN cd /home/ifm/catkin_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make

# Multistage build to reduce image size
ARG BASE_IMAGE
FROM ${BASE_IMAGE}:${FINAL_IMAGE_TAG}
# Copy files built in previous stage
COPY --from=build /home/ifm/catkin_ws /home/ifm/catkin_ws
COPY --from=build /home/ifm/ifm3d/*.deb /home/ifm/ifm3d/
WORKDIR /home/ifm

# Install ifm3d and ifm3d-ros2 runtime dependencies
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    libxmlrpc-c++8v5 \
    locales \
    sudo \
    libssl-dev \
    libgoogle-glog0v5 \    
    libboost-all-dev \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install ifm3d
RUN cd /home/ifm/ifm3d &&\
    dpkg -i *.deb

RUN cd /home/ifm/catkin_ws && \
    apt-get update && \
    rosdep init && \
    rosdep update --rosdistro=${ROS_DISTRO} && \
    rosdep install --from-path src -y --ignore-src

# Setup localisation
RUN echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
    locale-gen en_US.UTF-8 && \
    /usr/sbin/update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# NOTE: Make sure to run export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
