ifm3d-ros on Ubuntu 18.04 and ROS Melodic
-----------------------------------------

NOTE: The instructions below only apply if you plan to install `ifm3d` from
source. You can, as of version 0.12.0, install `ifm3d` from binary
debs. Instructions for that are located at the main
[ifm3d project page](https://github.com/ifm/ifm3d).


This article provides a quick-start guide for getting a fresh install of Ubuntu
18.04 ready for usage with `ifm3d-ros` and an O3D camera. As a pre-requisite
for this article, we assume you already have Ubuntu 18.04 installed (but have
done no other configuration). A *minimal* install of 18.04 is sufficient for
following along below.

### Update the Baseline Packages of your Ubuntu 18.04 Install

```
$ sudo apt-get update
$ sudo apt-get -u upgrade
```

### Install ROS Melodic

You should now follow
[these steps](http://wiki.ros.org/melodic/Installation/Ubuntu) exactly (we
assume you did) and that you chose to install `ros-melodic-desktop-full`. Go do
that now, then continue on.

### Additional Dependencies

There are a few things that we need to install to successfully build from
source that we did not get implicitly by installing ROS. The following commands
will handle these pre-requisites:

```
$ sudo apt-get install libxmlrpc-c++8-dev
$ sudo apt-get install libgoogle-glog-dev
```

### Install ifm3d

[ifm3d](https://github.com/lovepark/ifm3d) is the core underlying C++ driver
that `ifm3d-ros` wraps. We need to install that now. We assume you keep all of
your source code in `~/dev`.

```
$ mkdir ~/dev
$ cd ~/dev
$ git clone https://github.com/lovepark/ifm3d.git
$ cd ifm3d
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr ..
$ make
$ make check
$ make package
$ make repackage
$ sudo dpkg -i ifm3d_0.9.0_amd64-camera.deb
$ sudo dpkg -i ifm3d_0.9.0_amd64-framegrabber.deb
$ sudo dpkg -i ifm3d_0.9.0_amd64-image.deb
$ sudo dpkg -i ifm3d_0.9.0_amd64-tools.deb
```

You are now in position to install `ifm3d-ros`. Those instructions are
available [here](building.md).
