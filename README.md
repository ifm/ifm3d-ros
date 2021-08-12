ifm3d-ros wrapper around the ifm3d C++ software
=========
ifm3d-ros is a wrapper around [ifm3d](https://github.com/lovepark/ifm3d)
enabling the usage of ifm pmd-based ToF cameras from within
[ROS](http://ros.org) software systems.

>TODO: add a 3D-image which is easier to understand
<!-- ![rviz1](doc/figures/rviz_sample.png) -->

Software Compatibility Matrix
=============================

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


Building and Installing the Software
====================================

1. Preparing your system: [Noetic](doc/noetic.md), [Melodic](doc/melodic.md)
2. [Installing the ROS node](doc/building.md)

ROS Interface
=============

## camera nodelet

The core `ifm3d-ros` sensor interface is implemented as a ROS nodelet. This allows for lower-latency data processing vs. the traditional out-of-process node-based ROS interface for applications that require it. However, we ship a launch file with this package that allows for using the core `ifm3d-ros` driver
as a standard node. To launch the node, the following command can be used:

```
$ roslaunch ifm3d camera.launch
```

We note, the above command is equivalent to the following and is used for purposes of providing a backward compatible interface to versions of `ifm3d-ros` prior to the conversion to a nodelet architecture:

```
$ roslaunch ifm3d nodelet.launch __ns:=ifm3d
```

Regardless of which command line you used from above, the launch file(s) encapsulate several features:  

1. It (partially) exposes the `camera_nodelet` parameters as command-line arguments for ease of runtime configuration.
2. It instantiates a nodelet manager which the `camera_nodelet` will be loaded into.
3. It launches the camera nodelet itself.
4. It publishes the static transform from the camera's optical frame to a traditional ROS sensor frame as a tf2 `static_transform_publisher`.

You can either use [this launch file](launch/camera.launch) directly, or, use it as a basis for integrating `ifm3d-ros` into your own robot software system.

### Known limitations of the software trigger use and verbose GLOG 

> NOTE: test this with a O3R camera: Question how can this be fixed with software / hardware triggers in the future?  

We note: due to the change in architecture from a standalone node to a nodelet, we have seen one behavior whose solution is not clear to have us provide backward compatible behavior with older versions of `ifm3d-ros`. Specifically, if you plan to run the camera in software triggered mode, you should launch then node as follows:

```
$ GLOG_minloglevel=3 roslaunch ifm3d camera.launch assume_sw_triggered:=true
```

The incompatibility here is that in prior versions, one would not need to explicitly set the `GLOG_` environment variable. While not strictly necessary, it is recommended for keeping the noise level of the `ifm3d` logs low.


### Parameters

| Name | Data Type | Default Value | Description |
| ---- | ---- | ---- | ---- |
| ~assume_sw_triggered | bool | false | This provides a hint to the driver that the camera is configured for software triggering (as opposed to free running). In this mode, certain default values are applied to lessen the noise in terms of timeouts from the frame grabber.|
| ~frame_id_base |string |ifm3d/camera | This string provides a prefix into the `tf` tree for `ifm3d-ros` coordinate frames. |
| ~frame_latency_thresh | float | 60.0 | Time (seconds) used to determine that timestamps from the camera cannot be trusted. When this threshold is exceeded, when compared to system time, we use the reception time of the frame and not the capture time of the frame. |
| ~ip | string | 192.168.0.69 | The IP address of the VPU. |
| ~password | string | "" | The password required to establish an edit session on the VPU |
| ~schema_mask | uint16 | 0xf |  The pcic schema mask to apply to the active session with the frame grabber. This determines which images are available for publication from the camera. More about pcic schemas can be gleaned from the [ifm3d projects documentation](https://www.ifm3d.com). |
| ~timeout_millis | int | 500 | The number of milliseconds to wait for the framegrabber to return new frame data before declaring a "timeout" and to stop blocking on new data. |
| ~timeout_tolerance_secs |float | 5.0 | The wall time to wait with no new data from the camera before trying to establish a new connection to the camera. This helps to providerobustness against camera cables becoming unplugged or other in-field pathologies which would cause the connection between the ROS node and the camera to be broken. |
| ~sync_clocks | bool | false | Attempt to sync the camera clock to the system clock at start-up. The side-effect is that timestamps on the image should reflect the capture time as opposed to the receipt time. |
| ~xmlrpc_port | unint16 | 80 | The TCP port the camera's xmlrpc server is listening on for requests. |
| ~pcic_port | unint16 | 50010 | The TCP (data) port the camera's pcic server is listening on for requests. |


### Published Topics

| Name | Data Type | Description |
| --- | --- | --- |
| amplitude | sensor_msgs/Image | The normalized amplitude image. |
| confidence | sensor_msgs/Image | The confidence image. |
| cloud | sensor_msgs/PointCloud2 | The point cloud data, i.e. X-, Y-, Z-coordinates. |
| distance | sensor_msgs/Image | The radial distance image. |
| raw_amplitude | sensor_msgs/Image | The raw (non normalized) amplitude image. |
| good_bad_pixels | sensor_msgs/Image | The binary image representation of the confidence image. |
| xyz_imaege | sensor_msgs/Image | A 3-channel image encoding of the point cloud. Each of the three image channels represents a spatial data plane encoding the x, y, z Cartesian values respectively. |
| unit_vectors | sensor_msgs/Image | The rotated unit vectors. |
| extrinsics | ifm3d/Extrinsics | The extrinsic calibration of the camera with respect to the camera optical frame. This 3D pose is encoded in mm and rad. |

>TODO: check if we keep the unit vectors. Currently, they are not implemented by the imf3d and therefore not published.   
>TODo: publish intrinsic values as well similar to the extrinsic parameters.  


### Subscribed Topics
None.

### Advertised Services

| Name | Service Definition | Description |
| ---- | ---- | ---- |
| Dump | ifm3d/Dump | Dumps the state of the camera system as a JSON (formatted as a string) |
| Config | ifm3d/Config | Provides a means to configure the VPU and Heads (imager settings), declaratively from a JSON (string) encoding of the desired settings. |
| SoftOff | ifm3d/SoftOff | Sets the active application of the camera into software triggered mode which will turn off the active illumination reducing both power and heat. |
| SoftOn | ifm3d/SoftOn | Sets the active application of the camera into free-running mode. Its intention is to act as the inverse of `SoftOff`. |
| SyncClocks | ifm3d/SyncClocks | Synchronizes the camera clock to the system time. The side-effect is that images can be stamped with the capture time of the frame as opposed to the reception time. |
| Trigger | ifm3d/Trigger | Requests the driver to software trigger the imager for data acquisition. |

> TODO: check how implement / use the concept of starting and triggering in the future without applications.   
> TODO: check how to implement triggers: distinction of software and hardware trigger.   



Additional Documentation
========================

* [Inspecting and configuring the camera/imager settings](doc/dump_and_config.md)
* [Troubleshooting](doc/troubleshooting.md)

LICENSE
-------

Please see the file called [LICENSE](LICENSE).
