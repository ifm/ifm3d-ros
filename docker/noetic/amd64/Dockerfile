FROM ros:noetic-perception AS build

ARG ROS_DISTRO=noetic
ARG LSB_RELEASE=focal
ARG CMAKE_VERSION=3.20.6
ARG IFM3D_TAG=tags/v0.20.3
ARG IFM3D_ROS_BRANCH="v0.7.1"
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


# clone and install ifm3d frmom tag
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

# copy ifm3d from install location to /usr
RUN cp -r /install/* /usr/

# Initialize catkin workspace
RUN mkdir -p catkin_ws/ifm3d-ros/src
RUN /bin/bash -c 'cd catkin_ws/ifm3d-ros/src; . /opt/ros/${ROS_DISTRO}/setup.bash; catkin_init_workspace'

# Clone and build ifm3d-ros repo
ARG IFM3D_ROS_CLONE_REPO
RUN cd /home/ifm/catkin_ws/ifm3d-ros/src && \
    git clone ${IFM3D_ROS_CLONE_REPO} -b ${IFM3D_ROS_BRANCH} --single-branch 

RUN /bin/bash -c 'cd catkin_ws/ifm3d-ros; . /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make'

# # create a temporary install location for catkin packages
RUN mkdir -p /home/ifm/catkin_ws/ifm3d-ros/
RUN /bin/bash -c 'cd /home/ifm/catkin_ws/ifm3d-ros/; . /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make install -DCMAKE_INSTALL_PREFIX=/home/ifm/catkin_ws/ifm3d-ros/install'

# multistage and switch to bare ros image
FROM ros:noetic-ros-core

ARG DEBIAN_FRONTEND=noninteractive
COPY --from=build /install/ /usr/
COPY --from=build /home/ifm/catkin_ws/ /home/ifm/catkin_ws/

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    libgoogle-glog0v5 \
    libopencv-core* \
    libxmlrpc-c++8v5 \
    locales \
    sudo \
    && apt-get clean

# install additional run dependencies of ifm3d-ros
RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-nodelet \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-robot=1.5.0-1* \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean && rm -rf /var/lib/apt/lists/*s \
    && apt-get autoremove -y \
    && dpkg -r --force-depends perl-modules-5.30 gfortran-8 perl-modules-5.30 humanity-icon-theme \
    && dpkg -r --force-depends libicu-dev \
    && dpkg -r --force-depends libpython3.8-dev cmake-data libapr1-dev libgcc-7-dev \
    libmysqlclient-dev libstdc++-7-dev libc6-dev cmake perl-modules-5.30 libpython3.8-dev libperl5.30 \
    && dpkg -r --force-depends libgl1-mesa-dri

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