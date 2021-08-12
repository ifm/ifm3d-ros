# Building and Installing the ifm3d-ros Package
---------------------------------------------

## Prerequisites

Use either of these two ROS distributions:  
1. [Ubuntu 20.04 LTS](http://www.ubuntu.com)
2. [ROS Noetic](http://www.ros.org/install) - we recommend `ros-noetic-desktop-full`.
3. [ifm3d](https://github.com/ifm/ifm3d) - version >= TODO: add 

or

1. [Ubuntu 18.04 LTS](http://www.ubuntu.com)
2. [ROS Melodic](http://www.ros.org/install) - we recommend `ros-melodic-desktop-full`.
3. [ifm3d](https://github.com/ifm/ifm3d) - version >= 0.9.2

> Note: Some users may require older ROS distributions for legacy reasons. The supplied ROS package may very well work with limited changes on older ROS distributions. At least previous version could be run as far back as Indigo and Kinetic. However, we didn't test this ourselves. Please be aware that if you chose to go this route we can no guarantee is given.

> TODÒ: test the 'new' ifm3d-ros wrapper in a Docker container for Ubuntu 18.04 and ROS Melodic.


## Step-by-Step Build Instructions

Step-by-step instructions on getting a fresh installation of Ubuntu and ROS prepared for usage with `ifm3d-ros` are available at the following links:
* [Ubuntu 20.04 with ROS Noetic](noetic.md)
* [Ubuntu 18.04 with ROS Melodic](melodic.md)

Building and installing ifm3d-ros is accomplished by utilizing the ROS [catkin](http://wiki.ros.org/catkin) tool. There are many tutorials and other pieces of advice available on-line advising how to most effectively utilize catkin. However, the basic idea is to provide a clean separation between your source code repository and your build and runtime environments. The instructions that now follow represent how we choose to use catkin to build and _permanently install_ a ROS package from source.  

### 1. Installation site of ros
First, we need to decide where we want our software to ultimately be installed. For purposes of this document, we will assume that we will install our ROS packages at `~/ros`.    

>NOTE: Below we assume `noetic`. Adapting to other ROS distributions is left as an exercise for the reader.

```
if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi
```
### 2. Get the ros wrapper code from Github
Next, we need to get the code from Github. We assume we keep all of our git repositories in `~/dev`.

```
$ cd ~/dev
$ git clone https://github.com/ifm3d/ifm3d-ros.git
```

### 3. create and initialize your catkin workspace 
We now have the code in `~/dev/ifm3d-ros`. Next, we want to create a _catkin workspace_ that we can use to build and install that code from. It is the catkin philosophy that we do not do this directly in the source directory.
```
$ cd ~/catkin
$ mkdir ifm3d
$ cd ifm3d
$ mkdir src
$ cd src
$ catkin_init_workspace
$ ln -s ~/dev/ifm3d-ros ifm3d
```

So, you should have a catkin workspace set up to build the ifm3d-ros code that looks similar to this:
```
[ ~/catkin/ifm3d/src ]
rosuser@tuna: $ pwd
/home/rosuser/catkin/ifm3d/src

[ ~/catkin/ifm3d/src ]
rosuser@tuna: $ ls -l
total 0
lrwxrwxrwx 1 rosuser rosuser 50 Mar 26 15:16 CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
lrwxrwxrwx 1 rosuser rosuser 31 Mar 26 15:16 ifm3d -> /home/rosuser/dev/ifm3d-ros
```

### 4. build the ros node code  
Now we are ready to build the code. The following code block shows you how to simply run catkin_make without anything else happening further. 
```
$ cd ~/catkin/ifm3d
$ catkin_make -DCATKIN_ENABLE_TESTING=OFF
```
This will create a `devel` folder in your catkin workspace, which contains the required code for running the ros node. To test this you can easily set-up your current shell and run: `source ~/catkin/ifm3d/devel/setup.bash && roslaunch ifm3d camera.launch`.  


Alternatively we supply a set of test scripts for testing the ifm3d-ros node after compiling. Please be aware that the following example code will also immediately install the ifm3d-ros node code to your system.
```
$ cd ~/catkin/ifm3d
$ catkin_make -DCATKIN_ENABLE_TESTING=ON
$ catkin_make run_tests
$ catkin_make -DCMAKE_INSTALL_PREFIX=${HOME}/ros/ifm3d install
```

The ROS package should now be installed in `~/ros/ifm3d`. To test everything out you should open a fresh bash shell, and start up a ROS core:   

> Note: please remember to source the required `setup.bash` files for every shell, if you haven't included that in your `.bashrc`.  

```
$ roscore
```

Open another shell and start the primary camera node:
```
$ roslaunch ifm3d camera.launch
```

Open another shell and start the rviz node to visualize the data coming from
the camera:
```
$ roslaunch ifm3d rviz.launch
```

At this point, you should see an rviz window that looks something like:
> TODO: replace this meaningless / hard to interpret screenshot of a pallet with something cool.

<!-- ![rviz1](figures/rviz_sample.png) -->

Congratulations! You can now utilize ifm3d-ros.
