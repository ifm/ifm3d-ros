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

## How to call the custom services
We implemented custom services for transferring JSON strings to configure the whole camera platform. They can be used as partial or full JSON strings.

### Dump
Example use:
```
rosservice call /ifm3d_ros_examples/camera/Dump
```

### Config
Example use:
```
rosservice call /ifm3d_ros_examples/camera/Config '"{\"ports\":{\"port2\":{\"mode\":\"standard_range2m\"}}}"'
```


### Using the helper command line tools

We provide helper tools that mimic the command line interface of ifm3d. They are working assuming that a single camera node is running from the `ifm3d_ros_examples` package (`roslaunch ifm3d_ros_examples camera.launch`):
```
$ rosrun ifm3d_ros_msgs dump | jq '.ports.port0.state="CONF"' | rosrun ifm3d_ros_msgs config
$ rosrun ifm3d_ros_msgs dump | jq .ports.port0.state
"CONF"
```

# LICENSE
Please see the file called [LICENSE](LICENSE).