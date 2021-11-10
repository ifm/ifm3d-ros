# ifm3d-ros wrapper around the ifm3d library


**NOTE: The ifm3d-ros package has had major changes recently. Please be aware that this might cause problems on your system for building pipelines based on our old build instructions.**  
We tried to ensure backward compatibility where ever possible. If you find any major breaks, please let us know.  


ifm3d-ros is a wrapper around [ifm3d](https://github.com/ifm/ifm3d/) enabling the usage of the O3R camera platform (ifm ToF cameras) from within [ROS](http://ros.org) software systems.  

![rviz1](ifm3d_ros_driver/doc/figures/O3R_merged_point_cloud.png)

## Software Compatibility Matrix

|**ifm3d-ros version**|**ifm3d version**|**ROS distribution(s)**|
|---|---|---|
| 0.1.0 | 0.1.0 | Kinetic |
| 0.2.0 | 0.2.0 | Kinetic, Indigo |
| 0.3.0 | 0.2.0 | Kinetic, Indigo |
| 0.4.0 | 0.3.0, 0.3.1, 0.3.2 | Kinetic, Indigo |
| 0.4.1 | 0.3.3, 0.4.0, 0.5.0, 0.6.0, 0.7.0, 0.8.1 | Kinetic, Indigo |
| 0.4.2 | 0.9.0 | Kinetic |
| 0.5.0 | 0.9.0, 0.9.1 | Kinetic |
| 0.5.1 | 0.9.2 | Kinetic, Melodic |
| 0.6.0 | 0.9.2, 0.9.3, 0.10.0, 0.11.0, 0.11.2 | Kinetic, Melodic |
| 0.6.1, 0.6.2 | 0.11.2, 0.12.0, 0.17.0 | Kinetic, Melodic |
| 0.7.0 | NN - in dev - placeholder 0.91.0 | Noetic |


## Organization of the software

The `ifm3d-ros` meta package provides three subpackages:
- `ifm3d_ros_driver` provides the core interface for receiving data for ifm 3d (O3R) cameras. 
- `ifm3d_ros_msgs` gathers the ifm-specific messages types and the services for configuring and triggering the camera.
- `ifm3d_ros_examples` provides additional helper scripts and examples.

The name `ifm3d-ros`  was kept even tough this is not consistent with ROS package naming conventions.   
This ROS package has been split into three sub packages in an effort to facilitate dependency handling on distributed systems and simplify deployment on embedded platforms. For instance, the package `ifm3d_ros_msgs` can be installed independently of the other packages to control the camera from a separate computing platform. The `ifm3d_ros_examples` holds our launch files and examples.

## Building and installing the software

1. Preparing your system: [Noetic](ifm3d_ros_driver/doc/noetic.md)
2. [Installing the ifm3d-ros node](ifm3d_ros_driver/doc/building.md)

# LICENSE
Please see the file called [LICENSE](LICENSE).