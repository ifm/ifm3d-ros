# Building and installing for distributed systems

We have structured the ROS node into separate subpackages. This allows us to build only a subset of packages from the metapackage `ifm3d-ros`.

## Building one package independently from your catkin workspace
The command below allows you to selectively build only one of the existing packages in your joined catkin workspace. All the following commands can only be called on a device which already has ROS installed.


```
$ catkin_make --only-pkg-with-deps <target_package>
```
To apply this to one of the ifm3d-ros subpackages, please replace `<target_package>` with either one item of the following list:  
- `ifm3d_ros_driver` provides the core interface for receiving data for ifm 3d (O3R) cameras. 
- `ifm3d_ros_msgs` gathers the ifm-specific messages types and the services for configuring and triggering the camera.
- `ifm3d_ros_examples` provides additional helper scripts and examples.

### Building the driver package
Let's say you want to run the `ìfm3d_ros_driver` on an embedded system like the VPU itself. To make it as lightweight as possible we suggest to only build the subpackage `ifm3d_ros_driver` and its dependencies on / for the desired architecture. This will look something like the following:
```
$ catkin_make --only-pkg-with-deps ifm3d_ros_driver

traversing 2 packages in topological order:
-- ~~  - ifm3d_ros_msgs
-- ~~  - ifm3d_ros_driver
```

### Building the messages package
Typically a secondary ROS client would only subscribe to topics of a shared ROS master for receiving data and configure the main driver. Both jobs can be achieved without having the main driver package `ifm3d_ros_driver` installed on the client system. Therefor only compile the package `ifm3d_ros_msgs`.

```
$ catkin_make --only-pkg-with-deps ifm3d_ros_msgs

traversing 1 packages in topological order:
-- ~~  - ifm3d_ros_msgs
```

### Building the examples package
Building the examples package by itself (with dependencies) will result in having all packages build as it requires all three. 
```
$ catkin_make --only-pkg-with-deps ifm3d_ros_examples

traversing 3 packages in topological order:
-- ~~  - ifm3d_ros_msgs
-- ~~  - ifm3d_ros_driver
-- ~~  - ifm3d_ros_examples

```