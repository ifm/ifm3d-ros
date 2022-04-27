# The `ifm3d_ros_driver` package

## ROS Interface

The core `ifm3d-ros` sensor interface is implemented as a ROS nodelet. This allows for lower-latency data processing vs. the traditional out-of-process node-based ROS interface for applications that require it. However, we ship a launch file with this package that allows for using the core `ifm3d-ros` driver as a standard node. To launch the node, the following command can be used:

```
$ roslaunch ifm3d_ros_examples camera.launch
```
>Note: Please notice the use of the subpackage `ifm3d_ros_examples`.

For further information about the internal ROS nodelet infrastructure and how to apply this to your application, please see our exemplary launch files and a short run down on the nodelet structure in the [ifm3d_ros_examples/README](../ifm3d_ros_examples/README.md).

### Nodelet - parameters

| Name | Data Type | Default Value | Description |
| ---- | ---- | ---- | ---- |
| ~assume_sw_triggered | bool | false | This provides a hint to the driver that the camera is configured for software triggering (as opposed to free running). In this mode, certain default values are applied to lessen the noise in terms of timeouts from the frame grabber.|
| ~frame_id_base |string |ifm3d/camera | This string provides a prefix into the `tf` tree for `ifm3d_ros` coordinate frames. |
| ~frame_latency_thresh | float | 60.0 | Time (seconds) used to determine that timestamps from the camera cannot be trusted. When this threshold is exceeded, when compared to system time, we use the reception time of the frame and not the capture time of the frame. |
| ~ip | string | 192.168.0.69 | The IP address of the VPU. |
| ~password | string | "" | The password required to establish an edit session on the VPU |
| ~schema_mask | uint16 | 0xf |  The pcic schema mask to apply to the active session with the frame grabber. This determines which images are available for publication from the camera. More about pcic schemas can be gleaned from the [ifm3d projects documentation](https://www.ifm3d.com). |
| ~timeout_millis | int | 500 | The number of milliseconds to wait for the framegrabber to return new frame data before declaring a "timeout" and to stop blocking on new data. |
| ~timeout_tolerance_secs |float | 5.0 | The wall time to wait with no new data from the camera before trying to establish a new connection to the camera. This helps to providerobustness against camera cables becoming unplugged or other in-field pathologies which would cause the connection between the ROS node and the camera to be broken. |
| ~sync_clocks DEPRECATED | bool | false | Attempt to sync the camera clock to the system clock at start-up. The side-effect is that timestamps on the image should reflect the capture time as opposed to the receipt time. |
| ~xmlrpc_port | unint16 | 80 | The TCP port the camera's xmlrpc server is listening on for requests. |
| ~pcic_port | unint16 | 50010 | The TCP (data) port the camera's pcic server is listening on for requests. |


### Nodelet - published Topics

| Name | Data Type | Description |
| --- | --- | --- |
| amplitude | sensor_msgs/Image | The normalized amplitude image. |
| confidence | sensor_msgs/Image | The confidence image. |
| cloud | sensor_msgs/PointCloud2 | The point cloud data, i.e. X-, Y-, Z-coordinates. |
| distance | sensor_msgs/Image | The radial distance image. |
| raw_amplitude | sensor_msgs/Image | The raw (non normalized) amplitude image. |
| unit_vectors | sensor_msgs/Image | The rotated unit vectors. |
| extrinsics | ifm3d/Extrinsics | The extrinsic calibration of the camera with respect to the camera optical frame. This 3D pose is encoded in mm and rad. |
| rgb_image/compressed | sensor_msgs::CompressedImage | The RGB image in compressed format. |
>Note: Some topics may have empty data fields. We are working on publishing data on all available topics, but have kept all previous topics active for the moment for legacy reasons.   

### Nodelet - subscribed Topics
None.

### Nodelet - advertised Services
> Note: the services are provided by the `ifm3d_ros_msgs` package.

| Name | Service Definition | Description |
| ---- | ---- | ---- |
| Dump | ifm3d/Dump | Dumps the state of the camera system as a JSON (formatted as a string) |
| Config | ifm3d/Config | Provides a means to configure the VPU and Heads (imager settings), declaratively from a JSON (string) encoding of the desired settings. |
| SoftOff | ifm3d/SoftOff | Sets the active application of the camera into software triggered mode which will turn off the active illumination reducing both power and heat. |
| SoftOn | ifm3d/SoftOn | Sets the active application of the camera into free-running mode. Its intention is to act as the inverse of `SoftOff`. |
| Trigger | ifm3d/Trigger | Requests the driver to software trigger the imager for data acquisition. | 

### Known limitations 
[![O3R](https://img.shields.io/badge/O3R-lightgrey.svg)]()
[![O3D](https://img.shields.io/badge/O3D-green.svg)]()
[![O3X](https://img.shields.io/badge/O3X-green.svg)]()


## Additional Documentation
* [Inspecting and configuring the camera / imager settings](doc/dump_and_config.md)
* [Troubleshooting](doc/troubleshooting.md)

## LICENSE
Please see the file called [LICENSE](LICENSE).
