# ROS1 build required packages

# Install required ROS packages manually via apt package manager
**THIS IS THE SUGGESTED WAY TO GO**

Check the `package.xml` for each of your own ROS package for their dependencies. Many of the dependencies might not be required at runtime but only for building the package.   

The packages can be manually installed using this syntax:
```
apt-get update && apt-get install -y --no-install-recommends PACKAGENAME
```
This will ensure that only the required package will be installed. To find out which packages are available either ckeck
1. `aptitide search PACKAGENAME`
2. [archlinux package index](https://archlinux.org/packages/?)

When using the CLI of aptitude please make sure that the underlying architecture is the same as the requested architecture of your Docker image.  


Many packages come in dev versions such as `libopencv-dev` and also in runtime versions. Typically only the runtime versions are required in the final build stage of a Docker container.



# NOT SUGGESTED -  download ROS packages via build-essentials and rosdep 
If you are working on a 'bare-bones" ROS installation please keep in mind, that these Debian packages a required to build a ROS-package from source.

They can be downloaded using rosdep. I don't suggest to go this route as it is a bit cumbersome and ambiguous at the best of times. The user can decide which package will be installed explicitly and cat chose the version.

```
build-essential
```
To see more information on this topic see: [noetic/Installation/source](http://wiki.ros.org/noetic/Installation/Source).  
This will significantly increase your installation size. 

### add release packages
[kinetic adding release packages](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi#Adding_Released_Packages)

catkin_make 
```
root@d65f3c191098:/home/ifm/catkin_ws# catkin_make
Base path: /home/ifm/catkin_ws
Source space: /home/ifm/catkin_ws/src
Build space: /home/ifm/catkin_ws/build
Devel space: /home/ifm/catkin_ws/devel
Install space: /home/ifm/catkin_ws/install
####
#### Running command: "cmake /home/ifm/catkin_ws/src -DCATKIN_DEVEL_PREFIX=/home/ifm/catkin_ws/devel -DCMAKE_INSTALL_PREFIX=/home/ifm/catkin_ws/install -G Unix Makefiles" in "/home/ifm/catkin_ws/build"
####
-- The C compiler identification is GNU 9.3.0
-- The CXX compiler identification is GNU 9.3.0
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Detecting C compile features
-- Detecting C compile features - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Using CATKIN_DEVEL_PREFIX: /home/ifm/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /opt/ros/noetic
-- This workspace overlays: /opt/ros/noetic
-- Found PythonInterp: /usr/bin/python3 (found suitable version "3.8.10", minimum required is "3") 
-- Using PYTHON_EXECUTABLE: /usr/bin/python3
-- Using Debian Python package layout
-- Found PY_em: /usr/lib/python3/dist-packages/em.py  
-- Using empy: /usr/lib/python3/dist-packages/em.py
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/ifm/catkin_ws/build/test_results
-- Forcing gtest/gmock from source, though one was otherwise available.
-- Found gtest sources under '/usr/src/googletest': gtests will be built
-- Found gmock sources under '/usr/src/googletest': gmock will be built
-- Found PythonInterp: /usr/bin/python3 (found version "3.8.10") 
-- Found Threads: TRUE  
-- Using Python nosetests: /usr/bin/nosetests3
-- catkin 0.8.10
-- BUILD_SHARED_LIBS is on
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - ifm3d
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'ifm3d'
-- ==> add_subdirectory(ifm3d-ros)
-- ifm3d found component: camera
-- ifm3d found component: framegrabber
-- Found OpenCV: /usr (found version "4.2.0") found components: core 
-- Checking for module 'eigen3'
--   Found eigen3, version 3.3.7
-- Found Eigen: /usr/include/eigen3 (Required is at least version "3.1") 
-- Eigen found (include: /usr/include/eigen3, version: 3.3.7)
-- Found Boost: /usr/include (found suitable version "1.71.0", minimum required is "1.55.0") found components: system filesystem date_time iostreams regex 
-- looking for PCL_COMMON
-- Found PCL_COMMON: /usr/lib/aarch64-linux-gnu/libpcl_common.so  
-- Found PCL: pcl_common;/usr/lib/aarch64-linux-gnu/libboost_system.so;/usr/lib/aarch64-linux-gnu/libboost_filesystem.so;/usr/lib/aarch64-linux-gnu/libboost_date_time.so;/usr/lib/aarch64-linux-gnu/libboost_iostreams.so;/usr/lib/aarch64-linux-gnu/libboost_regex.so (Required is at least version "1.7") 
-- ifm3d found component: image
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- ifm3d: 1 messages, 6 services
-- Installing devel-space wrapper /home/ifm/catkin_ws/src/ifm3d-ros/bin/dump to /home/ifm/catkin_ws/devel/lib/ifm3d
-- Installing devel-space wrapper /home/ifm/catkin_ws/src/ifm3d-ros/bin/config to /home/ifm/catkin_ws/devel/lib/ifm3d
-- Configuring done
-- Generating done
-- Build files have been written to: /home/ifm/catkin_ws/build
```

### rosdep update without su 
ROS packages can be installed with rosdep. The problem when using it i a Docker build context are superuser rights. The argument `rosdep update` can not be run with sudo rights. Any attempts to rectify the user rights of ROS packages afterwards with `rosdep fix-permissions` results in nonworking ROS installation. I therefore suggest NOT to go this way.


1. create user wo su privileges
2. rosdep init 
3. rosdep update
```Recommended: please run

	rosdep update

Warning: running 'rosdep update' as root is not recommended.
  You should run 'sudo rosdep fix-permissions' and invoke 'rosdep update' again without sudo.
reading in sources list data from /etc/ros/rosdep/sources.list.d
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
Hit https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml
Query rosdistro index https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml
Skip end-of-life distro "ardent"
Skip end-of-life distro "bouncy"
Skip end-of-life distro "crystal"
Skip end-of-life distro "dashing"
Skip end-of-life distro "eloquent"
Add distro "foxy"
Add distro "galactic"
Skip end-of-life distro "groovy"
Skip end-of-life distro "hydro"
Skip end-of-life distro "indigo"
Skip end-of-life distro "jade"
Skip end-of-life distro "kinetic"
Skip end-of-life distro "lunar"
Add distro "melodic"
Add distro "noetic"
Add distro "rolling"
updated cache in /root/.ros/rosdep/sources.cache
```
