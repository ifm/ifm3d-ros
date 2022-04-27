# The `ifm3d_ros_msgs` package

This package provides the messages and services interfaces for the `ifm3d_ros_driver` package. It can be installed independently of the driver package `ifm3d_ros_driver` and examples package `ìfm3d_ros_examples`.

## Standalone Installation of the messages packages
If you plan on installing only one subpackage please see the instructions below. 

```
catkin_make --only-pkg-with-deps <target_package>
```
Please replace the tag `<target_package>` with the name of the package you want to compile:  
+ `ifm3d_ros_driver`
+ `ifm3d_ros_msgs`
+ `ifm3d_ros_examples`

Some of our subpackages have dependencies to other packages and therefore will trigger a compiling of more subpackages, namely the packages `ifm3d_ros_examples` and `ifm3d_ros_driver`. These subpackges can not be used standalone.

Don't forget to switch back to building all packages afterwards:  
```
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

## LICENSE
Please see the file called [LICENSE](LICENSE).