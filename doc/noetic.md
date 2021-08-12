ifm3d-ros on Ubuntu 20.04 and ROS Noetic
-----------------------------------------

NOTE: The instructions below only apply if you plan to install `ifm3d` from
source. You can, as of version 0.12.0, install `ifm3d` from binary
debs. However, we encourage the usage of Docker containers and download our Docker base image form Docker hub for quick testing.

For further information about the ifm3d O3R hardware and software please see our website.
[ifm3d.com developer website](https://ifm3d.com/).


This article provides a quick-start guide for getting a fresh installation of Ubuntu 20.04 ready for usage with `ifm3d-ros` and an O3R camera system. As a pre-requisite for this article, we assume you already have Ubuntu 20.04 installed (but have done no other configuration). A *minimal* installation of 20.04 is sufficient for
following along below.

1. Update the Baseline Packages of your Ubuntu 20.04:   
```
$ sudo apt-get update
$ sudo apt-get -u upgrade
```

2. Install ROS Melodic:  

You should now follow [these steps](http://wiki.ros.org/noetic/Installation/Ubuntu) exactly (we assume you did) and that you chose to install `ros-melodic-desktop-full`. 

3. additional Dependencies:    
There are a few things that we need to install to successfully build from source that we did not get implicitly by installing ROS. The following commands will handle these pre-requisites:
```
$ sudo apt-get install -y libboost-all-dev \
                       git \
                       libcurl4-openssl-dev \
                       libgtest-dev \
                       libgoogle-glog-dev \
                       libxmlrpc-c++8-dev \
                       libopencv-dev \
                       libpcl-dev \
                       libproj-dev \
                       python3-dev \
                       python3-pip \
                       build-essential \
                       coreutils \
                       findutils \
                       cmake \
                       locales \
                       ninja-build
```

4. Install ifm3d:  
[ifm3d](https://github.com/ifm/ifm3d) is the core underlying C++ driver that `ifm3d-ros` wraps. We need to install that beforehand. We assume you keep all of your source code in `~/dev`.  

```
$ mkdir ~/dev
$ cd ~/dev
$ git clone --branch o3r/main https://github.com/ifm/ifm3d
$ cd ifm3d
$ echo "Building from current branch" && \
$ mkdir build && \
$ cd build && \
$ cmake -GNinja -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_MODULE_OPENCV=ON -DBUILD_MODULE_PCICCLIENT=ON -DBUILD_MODULE_PYBIND11=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 .. && \
$ ninja && \
$ ninja package && \
$ ninja repackage
```

This instruction on how to install the imf3d from source is just a quick rundown of how we would typically do it. To get a more complete picture on the installation process please see the [ifm3d: building form source documentation](dummy-link). TODO add link.  

You are now in position to install the `ifm3d-ros` wrapper. Please switch to the instructions [here](building.md).
