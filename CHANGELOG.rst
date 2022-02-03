^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ifm3d-ros
^^^^^^^^^^^^^^^^^^^^^^^^^

1.0
===

1.0.0
------

Braking changes:
+ Restructure the ifm3d-ros package into independent subpackages. Please check your path declarations again, especially for the launch files and messages and services.

Changes between ifm3d_ros 0.6.x and 1.0.0:
+ Order of axis changed in 3D (cloud topic and extrinsic calibration parameters): This wrapper keeps the axis orientation as defined by the underlying API, ifm3d. Therefore, you may see a different axis order for the cloud message compared to older versions of the ifm3d and ifm cameras.
+ Extrinsic calibration parameters: now consistent with SI units, e.g. translation are scaled in `m` and rotation parameters are scaled in `rad`.
+ Added publisher for 2D RGB data
+ Use CameraBase for compatibility with other O3 devices
+ Comment out methods / publisher which are not available for the O3RCamera (at the moment)
+ Comment out the unit vector publishing
+ Changed services trigger and softon, softoff to be compatible with new JSON methods and schema.
+ Changed service trigger to a dummy method until triggers are implemented for the O3R platform. It only has a status message at the moment.
+ Changed service dump: coverts from json to str for displaying the message

added:
+ Added pcic_port to the list of framegrabber arguments

known limitations:
+ This version of the ifm3d-ros package only works with the O3R camera platform.


0.6
===

0.6.2
-----
Changes between ifm3d_ros 0.6.1 and 0.6.2

* Updated maintainer email address
* Added ifm3d-core dependency in preparation for submission to the ROS index

0.6.1
-----

* Added support syncing the system and camera clocks at startup. Side-effect,
  is we can now stamp the images with the camera-side capture time and not the
  host-side reception time.
* Added the `SyncClocks` Service

0.6.0.
------

* Added a image transport plugin _blacklist_ to the nodlet launch file. This
  prevents many of the errors seen in the terminal when running `rosbag -a` to
  capture camera data
* Added the `SoftOn` and `SoftOff` service calls

0.5
===


0.5.1
-----

* Added support for Ubuntu 18.04 and ROS Melodic

0.5.0
-----

* Converted primary data publisher to a nodelet architecture
* Provide the `dump` and `config` scripts to call into the exposed ROS services
  of the nodelet. Removed the older "config node".
* Added unit tests

0.4
===

0.4.2
-----

* Now requires ifm3d 0.9.0 and by association the more modernized tooling
  (C++14, cmake 3.5, dropped support for 14.04/Indigo, etc.)

0.4.1
-----

* Now publishing extrinsics on a topic

0.3
===

* Added `Dump` Service
* Added `Config` Service
* Added `Trigger` Service

0.2
===

* Updates to CMakeLists.txt to support Ubuntu 14.04 and ROS Indigo

0.1
===

This file has started tracking ifm3d_ros at 0.1.0

* Initial (alpha) release
