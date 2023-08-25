# Release notes ifm3d-ros

## Dependent software and firmware release notes and changelogs
+ ifm3d-ros: For changes between on the ifm3d-ros node source code, please see the respective information in the [CHANGELOG](CHANGELOG.rst).
+ ifm3d: For changes between on the ifm3d API source code, please see the respective information in the [ifm3d CHANGELOG](https://github.com/ifm/ifm3d/blob/main/ChangeLog.md).
+ O3R firmware: For changes between on the different FW versions, please see the respective information in the [FW release notes](https://ifm3d.com/documentation/Firmware/index.html).



## ifm3d-ros 1.1
The ifm3d-ros node version 1.1 depends on ifm3d API 1.2.6, as can be seen in the [compatibility matrix](README.md#software-compatibility-matrix).

Between ifm3d API version 0.93 and 1.1 the internal handling of the amplitude and distance image and the point cloud has changed: The previously automatically applied masking based on the confidence image is no longer applied. As a result, more image pixels are displayed as valid pixels.
If you want to apply a binary mapping to these images, use the confidence image as provided in the ROS node and apply the mask `confidence&1` for backward compatibility.