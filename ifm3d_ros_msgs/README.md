# The `ifm3d_ros_msgs` package

This package provides the messages and services interfaces for the `ifm3d_ros_driver` package. It can be installed independently of the driver package `ifm3d_ros_driver` and examples package `ìfm3d_ros_examples`.

## Standalone Installation of the messages packages
>TODO: elaborate on the installation alone

```
catkin_make --only-pkg-with-deps <target_package>
```

Don't forget to switch back to building all packages afterwards:  
```
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

# LICENSE
Please see the file called [LICENSE](LICENSE).