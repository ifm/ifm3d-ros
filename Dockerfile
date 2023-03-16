ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM $BASE_IMAGE:$BASE_IMAGE_TAG AS build

ARG ROS_DISTRO=noetic
ARG LSB_RELEASE=focal
ARG CMAKE_VERSION=3.20.6
ARG IFM3D_TAG=tags/v1.2.2

# Create the ifm user
RUN id ifm 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U ifm
WORKDIR /home/ifm

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    git \
    jq \
    libcurl4-openssl-dev \
    libgtest-dev libgoogle-glog-dev  \
    libxmlrpc-c++8-dev \
    libproj-dev \
    build-essential \
    coreutils \
    cmake \
    ninja-build \
    wget \
    libssl-dev\
    libboost-all-dev

RUN apt-get clean

# Install cmake
RUN wget -O - "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-$(uname -i).tar.gz" \
    | tar -xz --strip-components=1 -C /usr


# clone and install ifm3d frmom tag
ARG IFM3D_CLONE_REPO

# | Flag name | Description | Default value |
# | --------- | ----------- | ------------- |
# | BUILD_MODULE_FRAMEGRABBER | Build the framegrabber module | ON |
# | BUILD_MODULE_STLIMAGE | Build the stl image module (Only relies on standard c++ libraries) | OFF |
# | BUILD_MODULE_IMAGE **DEPRECATED**| Build the image module (Depends on OpenCV and PCL) | OFF |
# | BUILD_MODULE_OPENCV **DEPRECATED**| Build the OpenCV-only image container | OFF |
# | BUILD_MODULE_TOOLS | Build the command-line utility | ON |
# | BUILD_IN_DEPS | Download and build dependencies | ON |
# | BUILD_MODULE_PYBIND11 | Build the ifm3dpy python package (it can also be installed directly through `pip`) | OFF |
# | USE_LEGACY_COORDINATES | Use the legacy coordinates (ifm3d <= 0.92.x) with swapped axis | OFF |
# | BUILD_MODULE_SWUPDATER | Build the swupdater module | ON |
# | BUILD_SDK_PKG | Build install packages for development purposes | ON |
# | FORCE_OPENCV3 | Force the build to require OpenCV 3 | OFF |
# | FORCE_OPENCV2 | Force the build to require OpenCV 2.4 | OFF |
# | BUILD_SHARED_LIBS | Build modules as shared libraries | ON |
# | BUILD_EXAMPLES | Build the examples | OFF |
# | BUILD_DOC | Build documentation | OFF |
# | BUILD_TESTS | Build unit tests | ON
# | BUILD_MODULE_PCICCLIENT | Build the pcicclient module | OFF |

RUN cd /home/ifm/ \
    && git clone --branch v1.2.2 ${IFM3D_CLONE_REPO} ifm3d \
    && mkdir -p /home/ifm/ifm3d/build \
    && cd /home/ifm/ifm3d/build \
    && cmake -GNinja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/install \
    -DBUILD_MODULE_OPENCV=OFF \
    -DBUILD_MODULE_PCICCLIENT=ON \
    -DBUILD_MODULE_PYBIND11=OFF \
    -DBUILD_MODULE_TOOLS=ON\
    -DBUILD_MODULE_SWUPDATER=OFF\
    -DBUILD_SDK_PKG=ON\
    -DBUILD_EXAMPLES=OFF\
    -DBUILD_TESTS=OFF\
    .. \
    && cmake --build . \
    && cmake --build . --target install

RUN cp -r /install/* /usr

# # Modify cmake to remove bug: reference to OpenCV in ifm3d v0.93.0
# RUN sed -i.bak '22,44d' /usr/lib/cmake/ifm3d-0.93.0/ifm3d-config.cmake

# Initialize catkin workspace
RUN mkdir -p catkin_ws/ifm3d-ros/src
RUN /bin/bash -c 'cd catkin_ws/ifm3d-ros/src; . /opt/ros/${ROS_DISTRO}/setup.bash; catkin_init_workspace'

# # Clone and build ifm3d-ros repo
# ARG IFM3D_ROS_CLONE_REPO
# RUN cd /home/ifm/catkin_ws/ifm3d-ros/src && \
#     git clone ${IFM3D_ROS_CLONE_REPO}

RUN apt-get update && apt-get install -y nlohmann-json3-dev

ADD . /home/ifm/catkin_ws/ifm3d-ros/src
RUN cd /home/ifm/catkin_ws/ifm3d-ros/src
RUN /bin/bash -c 'cd catkin_ws/ifm3d-ros; . /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make'



# # multistage and switch to bare ros image
# ARG BASE_IMAGE
# ARG BASE_IMAGE_TAG

# FROM $BASE_IMAGE:$BASE_IMAGE_TAG

# ARG DEBIAN_FRONTEND=noninteractive
# COPY --from=build /install/ /usr/
# COPY --from=build /home/ifm/catkin_ws/ /home/ifm/catkin_ws/

# ARG ROS_DISTRO=noetic
# ARG DEBIAN_FRONTEND=noninteractive

# # Install runtime requirements
# RUN apt-get update \
#     && DEBIAN_FRONTEND=noninteractive apt-get install -y \
#         libgoogle-glog0v5 \
#         libxmlrpc-c++8v5 \
#         locales \
#         sudo \
#     && rm -rf /var/lib/apt/lists/*


# # install additional run dependencies of ifm3d-ros
# RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-nodelet \
#     ros-${ROS_DISTRO}-tf2-ros \
#     ros-${ROS_DISTRO}-robot=1.5.0-1* \
#     ros-${ROS_DISTRO}-image-transport \
#     && rm -rf /var/lib/apt/lists/* \
#     && apt-get clean && rm -rf /var/lib/apt/lists/*s \
#     && apt-get autoremove -y
#     # && dpkg -r --force-depends perl-modules-5.30 gfortran-8 perl-modules-5.30 humanity-icon-theme \
#     # && dpkg -r --force-depends libicu-dev \
#     # && dpkg -r --force-depends libpython3.8-dev cmake-data libapr1-dev libgcc-7-dev \
#     # libmysqlclient-dev libstdc++-7-dev libc6-dev cmake perl-modules-5.30 libpython3.8-dev libperl5.30 \
#     # && dpkg -r --force-depends libgl1-mesa-dri

# # Setup localisation
# RUN echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
#     locale-gen en_US.UTF-8 && \
#     /usr/sbin/update-locale LANG=en_US.UTF-8

# ENV LANG en_US.UTF-8
# ENV LANGUAGE en_US:en
# ENV LC_ALL en_US.UTF-8

# # Create the rosuser user
# RUN id rosuser 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U rosuser
# RUN echo "rosuser ALL=(ALL) NOPASSWD: ALL" | tee /etc/sudoers.d/rosuser

# # USER rosuser
# # COPY ./noetic/focal/ros_entrypoint.sh /
# #  ENTRYPOINT [ "/ros-entrypoint.sh" ]
