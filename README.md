# ifm3d-ros

## Release versions

:warning: Note that the `master` branch is generally in a work in progress, and you probably want to use a tagged [release version](https://github.com/ifm/ifm3d-ros/releases) for production.

## ifm3d-ros for the O3R

**NOTE: The ifm3d-ros package has had major changes recently. Please be aware that this might cause problems on your system for building pipelines based on our old build instructions.**
We tried to ensure backward compatibility where ever possible. If you find any major breaks, please let us know.


ifm3d-ros is a wrapper around [ifm3d](ifm3d/doc/sphinx/content/README:ifm3d%20Overview) enabling the usage of the O3R camera platform (ifm ToF cameras) from within [ROS](http://ros.org) software systems.
**Please make sure to use version {{ ifm3d_ros_latest_version }} or above for O3R compatibility.**

![rviz1](ifm3d_ros_driver/doc/figures/O3R_merged_point_cloud.png)

## Software Compatibility Matrix

|**ifm3d-ros version**|**ifm3d version**|**ROS distribution(s)**|
| ------------ | ------------ | ------------ |
| 1.0.0 | 0.91.0 | Noetic |
| 1.0.1 | 0.93.0 | Noetic |

### Internal ifm3d-ros subpackage version structure
Please see the internal subpackage version structure for a known `ifm3d-ros version`.

|**ifm3d-ros version**|**ifm3d_ros_driver**|**ifm3d_ros_msgs**|**ifm3d_ros_examples**|
| ------------ | ------------ | ------------ | ------------ |
| 1.0.0 | 0.7.0 | 0.1.0 | 0.1.0 |
| 1.0.1 | 1.0.1 | 0.1.0 | 0.1.0 |

## Organization of the software

The `ifm3d-ros` meta package provides three subpackages:
- `ifm3d_ros_driver` provides the core interface for receiving data for ifm 3d (O3R) cameras.
- `ifm3d_ros_msgs` gathers the ifm-specific messages types and the services for configuring and triggering the camera.
- `ifm3d_ros_examples` provides additional helper scripts and examples.

The name `ifm3d-ros`  was kept even tough this is not consistent with ROS package naming conventions.
This ROS package has been split into three sub packages in an effort to facilitate dependency handling on distributed systems and simplify deployment on embedded platforms. For instance, the package `ifm3d_ros_msgs` can be installed independently of the other packages to control the camera from a separate computing platform. The `ifm3d_ros_examples` holds our launch files and examples.

## Building and installing the software
:::{toctree}
:titlesonly:
Preparing your system <ifm3d_ros_driver/doc/noetic>
Installing the ifm3d-ros node <ifm3d_ros_driver/doc/building>
Build for distributed systems <ifm3d_ros_driver/doc/distributed_build>
:::
## LICENSE
Please see the file called [LICENSE](LICENSE).
