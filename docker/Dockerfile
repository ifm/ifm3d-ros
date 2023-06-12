ARG BASE_IMAGE
ARG ROS_DISTRO
ARG LSB_RELEASE
ARG CMAKE_VERSION=3.20.6
ARG IFM3D_TAG
ARG IFM3D_ROS_BRANCH

FROM ${BASE_IMAGE} AS build

# Create the ifm user
RUN id ifm 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U ifm
WORKDIR /home/ifm

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential \
    cmake \
    coreutils \
    findutils \
    git \
    libboost-all-dev \
    libcurl4-openssl-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    libopencv-dev \
    libpcl-dev \
    libproj-dev \
    libxmlrpc-c++8-dev \
    locales \
    ninja-build \
    wget \
    gpg
RUN apt-get clean

# Install cmake
RUN wget -O - "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-$(uname -i).tar.gz" \
    | tar -xz --strip-components=1 -C /usr


# First clone ifm3d repo
ARG IFM3D_CLONE_REPO

RUN mkdir src && \
    cd src && \
    git clone ${IFM3D_CLONE_REPO} ifm3d && \
    cd ifm3d && \
    git checkout ${IFM3D_TAG} && \
    echo "Building from tag ${IFM3D_TAG}" && \
    mkdir build && \
    cd build && \
    cmake -GNinja cmake \
    -DCMAKE_INSTALL_PREFIX=/install \
    -DBUILD_MODULE_IMAGE=ON \
    -DBUILD_IN_DEPS=OFF .. && \
    cmake --build . && \
    cmake --build . --target install

# copy ifm from install location to /usr
RUN cp -r /install/* /usr/

# use the first build stage to also compile the ros package as it has shared dependencies as ifm3d
RUN mkdir -p catkin_ws/ifm3d-ros/src

# Initialize catkin workspace
RUN /bin/bash -c 'cd catkin_ws/ifm3d-ros/src; . /opt/ros/${ROS_DISTRO}/setup.bash; catkin_init_workspace'

# Clone and build ifm3d-ros repo
ARG IFM3D_ROS_CLONE_REPO
RUN cd /home/ifm/catkin_ws/ifm3d-ros/src && \
    git clone ${IFM3D_ROS_CLONE_REPO} -b ${IFM3D_ROS_BRANCH} --single-branch 

RUN /bin/bash -c 'cd catkin_ws/ifm3d-ros; . /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make'

RUN mkdir -p /home/ifm/catkin_ws/ifm3d-ros/
RUN /bin/bash -c 'cd /home/ifm/catkin_ws/ifm3d-ros/; . /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make install -DCMAKE_INSTALL_PREFIX=/home/ifm/catkin_ws/ifm3d-ros/install'


FROM ros:melodic-ros-core

ARG DEBIAN_FRONTEND=noninteractive
COPY --from=build /install/ /usr/
COPY --from=build /home/ifm/catkin_ws/ /home/ifm/catkin_ws/

WORKDIR /home/ifm

RUN apt-get update && apt-get install -y --no-install-recommends \
    libgoogle-glog0v5 \
    libopencv-core* \
    libxmlrpc-c++8v5 \
    locales \
    sudo \
    && apt-get clean

# install additional run dependencies of ifm3d-ros
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-nodelet \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-robot\
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean && rm -rf /var/lib/apt/lists/*s \
    && apt-get autoremove -y

# Setup localisation
RUN echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
    locale-gen en_US.UTF-8 && \
    /usr/sbin/update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Create the rosuser user
RUN id rosuser 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U rosuser
RUN echo "rosuser ALL=(ALL) NOPASSWD: ALL" | tee /etc/sudoers.d/rosuser

USER rosuser

ADD ros-entrypoint.sh /ros-entrypoint.sh
ENTRYPOINT [ "/ros-entrypoint.sh" ]