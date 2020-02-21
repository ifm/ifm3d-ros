
=========
ifm3d-ros is a wrapper around [ifm3d](https://github.com/lovepark/ifm3d)
enabling the usage of ifm pmd-based ToF cameras from within
[ROS](http://ros.org) software systems.

![rviz1](doc/figures/rviz_sample.png)

Software Compatibility Matrix
=============================
<table>
  <tr>
    <th>ifm3d-ros version</th>
    <th>ifm3d version</th>
    <th>ROS distribution(s)</th>
  </tr>
  <tr>
    <td>0.1.0</td>
    <td>0.1.0</td>
    <td>Kinetic</td>
  </tr>
  <tr>
    <td>0.2.0</td>
    <td>0.2.0</td>
    <td>Kinetic, Indigo</td>
  </tr>
  <tr>
    <td>0.3.0</td>
    <td>0.2.0</td>
    <td>Kinetic, Indigo</td>
  </tr>
  <tr>
    <td>0.4.0</td>
    <td>0.2.0, 0.3.0, 0.3.1, 0.3.2</td>
    <td>Kinetic, Indigo</td>
  </tr>
  <tr>
    <td>0.4.1</td>
    <td>0.3.3, 0.4.0, 0.5.0, 0.6.0, 0.7.0, 0.8.1</td>
    <td>Kinetic, Indigo</td>
  </tr>
  <tr>
    <td>0.4.2</td>
    <td>0.9.0</td>
    <td>Kinetic</td>
  </tr>
  <tr>
    <td>0.5.0</td>
    <td>0.9.0, 0.9.1</td>
    <td>Kinetic</td>
  </tr>
  <tr>
    <td>0.5.1</td>
    <td>0.9.2</td>
    <td>Kinetic, Melodic</td>
  </tr>
  <tr>
    <td>0.6.0</td>
    <td>0.9.2, 0.9.3, 0.10.0, 0.11.0, 0.11.2</td>
    <td>Kinetic, Melodic</td>
  </tr>
  <tr>
    <td>0.6.1, 0.6.2</td>
    <td>0.11.2, 0.12.0, 0.17.0</td>
    <td>Kinetic, Melodic</td>
  </tr>
</table>

Building and Installing the Software
====================================

1. Preparing your system: [Kinetic](doc/kinetic.md), [Melodic](doc/melodic.md)
2. [Installing the ROS node](doc/building.md)

ROS Interface
=============

## camera nodelet

The core `ifm3d-ros` sensor interface is implemented as a ROS nodelet. This
allows for lower-latency data processing vs. the traditional out-of-process
node-based ROS interface for applications that require it. However, we ship a
launch file with this package that allows for using the core `ifm3d-ros` driver
as a standard node. To launch the node, the following command can be used:

```
$ roslaunch ifm3d camera.launch
```

We note, the above command is equivalent to the following and is used for
purposes of providing a backward compatible interface to versions of
`ifm3d-ros` prior to the conversion to a nodelet architecture:

```
$ roslaunch ifm3d nodelet.launch __ns:=ifm3d
```

Regardless of which command line you used from above, the launch file(s)
encapsulate several features:

1. It exposes some of the `camera_nodelet` parameters as command-line arguments
   for ease of runtime configuration.
2. It instantiates a nodelet manager which the `camera_nodelet` will be loaded
   into.
3. It launches the camera nodelet itself.
4. It publishes the static transform from the camera's optical frame to a
   traditional ROS sensor frame as a tf2 `static_transform_publisher`.

You can either use [this launch file](launch/camera.launch) directly, or, use
it as a basis for integrating `ifm3d-ros` into your own robot software system.

We note: due to the change in architecture from a standalone node to a nodelet,
we have seen one behavior whose solution is not clear to have us provide
backward compatible behavior with older versions of `ifm3d-ros`. Specifically,
if you plan to run the camera in software triggered mode, you should lanch the
node as follows:

```
$ GLOG_minloglevel=3 roslaunch ifm3d camera.launch assume_sw_triggered:=true
```

The incomatibility here is that in prior versions, one would not need to
explicitly set the `GLOG_` environment variable. While not strictly necessary,
it is recommended for keeping the noise level of the `ifm3d` logs low.


### Parameters

<table>
  <tr>
    <th>Name</th>
    <th>Data Type</th>
    <th>Default Value</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>~assume_sw_triggered</td>
    <td>bool</td>
    <td>false</td>
    <td>
      This provides a hint to the driver that the camera is configured for
      software triggering (as opposed to free running). In this mode, certain
      default values are applied to lessen the noise in terms of timeouts from
      the framegrabber.
    </td>
  </tr>
  <tr>
    <td>~frame_id_base</td>
    <td>string</td>
    <td>ifm3d/camera</td>
    <td>
      This string provides a prefix into the `tf` tree for `ifm3d-ros`
      coordinate frames.
    </td>
  </tr>
  <tr>
    <td>~frame_latency_thresh</td>
    <td>float</td>
    <td>60.0</td>
    <td>
      Time (seconds) used to determine that timestamps from the camera cannot
      be trusted. When this threshold is exceeded, when compared to system
      time, we use the reception time of the frame and not the capture time of
      the frame.
    </td>
  </tr>
  <tr>
    <td>~ip</td>
    <td>string</td>
    <td>192.168.0.69</td>
    <td>
      The ip address of the camera.
    </td>
  </tr>
  <tr>
    <td>~password</td>
    <td>string</td>
    <td></td>
    <td>
      The password required to establish an edit session with the camera.
    </td>
  </tr>
  <tr>
    <td>~schema_mask</td>
    <td>uint16</td>
    <td>0xf</td>
    <td>
      The pcic schema mask to apply to the active session with the frame
      grabber. This determines which images are available for publication from
      the camera. More about pcic schemas can be gleaned from the
      <a href="https://github.com/lovepark/ifm3d">ifm3d</a> project.
    </td>
  </tr>
  <tr>
    <td>~timeout_millis</td>
    <td>int</td>
    <td>500</td>
    <td>
      The number of milliseconds to wait for the framegrabber to return new
      frame data before declaring a "timeout" and to stop blocking on new
      data.
    </td>
  </tr>
  <tr>
    <td>~timeout_tolerance_secs</td>
    <td>float</td>
    <td>5.0</td>
    <td>
      The wall time to wait with no new data from the camera before trying to
      establish a new connection to the camera. This helps to provide
      robustness against camera cables becoming unplugged or other in-field
      pathologies which would cause the connection between the ROS node and the
      camera to be broken.
    </td>
  </tr>
  <tr>
    <td>~soft_on_timeout_millis</td>
    <td>int</td>
    <td>500</td>
    <td>
      If using the `SoftOn` service call, when turning the camera back `on`
      this is the setting that will be used for `timeout_millis`.
    </td>
  </tr>
  <tr>
    <td>~soft_on_timeout_tolerance_secs</td>
    <td>float</td>
    <td>5.0</td>
    <td>
      If using the `SoftOn` service call, when turning the camera back `on`
      this is the setting that will be used for `timeout_tolerance_secs`.
    </td>
  </tr>
  <tr>
    <td>~soft_off_timeout_millis</td>
    <td>int</td>
    <td>500</td>
    <td>
      If using the `SoftOff` service call, when turning the camera `off`
      this is the setting that will be used for `timeout_millis`.
    </td>
  </tr>
  <tr>
    <td>~soft_off_timeout_tolerance_secs</td>
    <td>float</td>
    <td>600.0</td>
    <td>
      If using the `SoftOff` service call, when turning the camera `off`
      this is the setting that will be used for `timeout_tolerance_secs`.
    </td>
  </tr>
  <tr>
    <td>~sync_clocks</td>
    <td>bool</td>
    <td>false</td>
    <td>
      Attempt to sync the camera clock to the system clock at start-up. The
      side-effect is that timestamps on the image should reflect the capture
      time as opposed to the receipt time.
    </td>
  </tr>
  <tr>
    <td>~xmlrpc_port</td>
    <td>uint16</td>
    <td>80</td>
    <td>
      The TCP port the camera's xmlrpc server is listening on for requests.
    </td>
  </tr>
</table>

### Published Topics

<table>
  <tr>
    <th>Name</th>
    <th>Data Type</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>amplitude</td>
    <td>sensor_msgs/Image</td>
    <td>The normalized amplitude image</td>
  </tr>
  <tr>
    <td>cloud</td>
    <td>sensor_msgs/PointCloud2</td>
    <td>The point cloud data</td>
  </tr>
  <tr>
    <td>confidence</td>
    <td>sensor_msgs/Image</td>
    <td>The confidence image</td>
  </tr>
  <tr>
    <td>distance</td>
    <td>sensor_msgs/Image</td>
    <td>The radial distance image</td>
  </tr>
  <tr>
    <td>extrinsics</td>
    <td><a href="msg/Extrinsics.msg">ifm3d/Extrinsics</a></td>
    <td>
      The extrinsic calibration of the camera with respect to the camera
      optical frame. The data are mm and degrees.
    </td>
  </tr>
  <tr>
    <td>good_bad_pixels</td>
    <td>sensor_msgs/Image</td>
    <td>
      A binary image indicating good vs. bad pixels as gleaned from the
      confidence data.
    </td>
  </tr>
  <tr>
    <td>raw_amplitude</td>
    <td>sensor_msgs/Image</td>
    <td>The raw amplitude image</td>
  </tr>
  <tr>
    <td>unit_vectors</td>
    <td>sensor_msgs/Image</td>
    <td>The rotated unit vectors</td>
  </tr>
  <tr>
    <td>xyz_image</td>
    <td>sensor_msgs/Image</td>
    <td>
      A 3-channel image encoding of the point cloud. Each of the three image
      channels respesent a spatial data plane encoding the x, y, z cartesian
      values respectively.
    </td>
  </tr>
</table>

### Subscribed Topics

None.

### Advertised Services

<table>
  <tr>
    <th>Name</th>
    <th>Service Definition</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>Dump</td>
    <td><a href="srv/Dump.srv">ifm3d/Dump</a></td>
    <td>Dumps the state of the camera parameters to JSON</td>
  </tr>
  <tr>
    <td>Config</td>
    <td><a href="srv/Config.srv">ifm3d/Config</a></td>
    <td>
      Provides a means to configure the camera and imager settings,
      declaratively from a JSON encoding of the desired settings.
    </td>
  </tr>
  <tr>
    <td>SoftOff</td>
    <td><a href="srv/SoftOff.srv">ifm3d/SoftOff</a></td>
    <td>
      Sets the active application of the camera into software triggered
      mode which will turn off the active illumination reducing both power and
      heat.
    </td>
  </tr>
  <tr>
    <td>SoftOn</td>
    <td><a href="srv/SoftOn.srv">ifm3d/SoftOn</a></td>
    <td>
      Sets the active application of the camera into free-running mode.
      Its intention is to act as the inverse of `SoftOff`.
    </td>
  </tr>
  <tr>
    <td>SyncClocks</td>
    <td><a href="srv/SyncClocks.srv">ifm3d/SyncClocks</a></td>
    <td>
      Synchronizes the camera clock to the system time. The side-effect is that
      images can be stamped with the capture time of the frame as opposed to
      the reception time.
    </td>
  </tr>
  <tr>
    <td>Trigger</td>
    <td><a href="srv/Trigger.srv">ifm3d/Trigger</a></td>
    <td>
      Requests the driver to software trigger the imager for data
      acquisition.
    </td>
  </tr>
</table>


Additional Documentation
========================

* [Inspecting and configuring the camera/imager settings](doc/dump_and_config.md)
* [Troubleshooting](doc/troubleshooting.md)

LICENSE
-------

Please see the file called [LICENSE](LICENSE).
