
# ROS logging for containers on embedded devices
By default Docker containers handle (verbose) logging in ways that is not suited well to space constrained systems, e.g. embedded devices.

There are a different options to reduce the chances of deadlocked systems because of (persistent) container logging:
+ If a (persistent) volume is mounted to a container, please be aware that the logs are persistent beyond the container lifetime. The data has to be cleaned up manually by the user.
+ Application specific logging configuration

For details on how configure specific logging drivers please see our [main documentation](https://ifm3d.com/).


## ROS logging configuration inside container
For details on how to set ROS specific logging, please see the details below [here](http://wiki.ros.org/rosconsole).

### Content of logging config file
Replace the content of the ROS logging config file with the following.
A change of the config file requires the user to source the setup.bash again: `source /opt/ros/$ROS_DISTRO/setup.bash`

**ROS logging file default location:** `$ROS_ROOT/config/rosconsole.config`
```
#
#   rosconsole will find this file by default at $ROS_ROOT/config/rosconsole.config
#
#   You can define your own by e.g. copying this file and setting
#   ROSCONSOLE_CONFIG_FILE (in your environment) to point to the new file
#
log4j.logger.ros=WARN
log4j.logger.ros.roscpp.superdebug=WARN
```


### Content of logging config file: ifm3d-ros specific configuration
For a ifm3d-ros node specific configuration, please use the config below:
```
log4j.logger.ros=INFO
log4j.logger.ros.roscpp.superdebug=WARN
log4j.logger.ros.ifm3d_ros_driver=WARN
log4j.logger.ros.ifm3d_ros_examples=WARN
log4j.logger.ros.ifm3d_ros_msgs=WARN
```