## Changes between ifm3d-ros 0.6.1 and 0.6.2

* Updated maintainer email address
* Added ifm3d-core dependency in preparation for submission to the ROS index

## Changes between ifm3d-ros 0.6.0 and 0.6.1

* Added support syncing the system and camera clocks at startup. Side-effect,
  is we can now stamp the images with the camera-side capture time and not the
  host-side reception time.
* Added the `SyncClocks` Service

## Changes between ifm3d-ros 0.5.1 and 0.6.0

* Added a image transport plugin _blacklist_ to the nodlet launch file. This
  prevents many of the errors seen in the terminal when running `rosbag -a` to
  capture camera data
* Added the `SoftOn` and `SoftOff` service calls

## Changes between ifm3d-ros 0.5.0 and 0.5.1

* Added support for Ubuntu 18.04 and ROS Melodic

## Changes between ifm3d-ros 0.4.2 and 0.5.0
* Converted primary data publisher to a nodelet architecture
* Provide the `dump` and `config` scripts to call into the exposed ROS services
  of the nodelet. Removed the older "config node".
* Added unit tests

## Changes between ifm3d-ros 0.4.1 and 0.4.2
* Now requires ifm3d 0.9.0 and by association the more modernized tooling
  (C++14, cmake 3.5, dropped support for 14.04/Indigo, etc.)

## Changes between ifm3d-ros 0.3.0 and 0.4.0

* Now publishing extrinsics on a topic

## Changes between ifm3d-ros 0.2.0 and 0.3.0

* Added `Dump` Service
* Added `Config` Service
* Added `Trigger` Service

## Changes between ifm3d-ros 0.1.0 and 0.2.0

* Updates to CMakeLists.txt to support Ubuntu 14.04 and ROS Indigo

## This file has started tracking ifm3d-ros at 0.1.0

* Initial (alpha) release
