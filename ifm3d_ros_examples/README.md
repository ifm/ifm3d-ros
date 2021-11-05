# The `ifm3d_ros_examples` package
This package provides examples and helper scripts for using the ifm O3R camera platform.


## Launchfiles

Please see the list below for launch files shipped with the examples package:  

| Name | Description |
| ---- | ----------- |
| `six_cameras.launch` | Launches six nodes, reading data streams on ports 0, 1, 2, 3, 4 and 5. Provide coordinate frame transforms for each node. Note: you can use this example for less than six heads, bur you will get a `timeout` error where no heads are connected. This does not disrupt the proper functioning of the connected heads.|
| `nodelet.launch` | This is handling the nodelet manager which makes it possible to launch a nodelet similarly as you would a simple node.|
| `head.launch` | Launches two data streams for both the 2D RGB imager and 3D TOF imager on a camera head. Please see /alter the launch file (for instructions) on how to connect your head |
| `camera.launch` | Launches a single camera stream - so only 3D data or 2D RGB data. This launch file is comparable to a single camera setup (O3Ds and O3Xs) | 

### Nodelet launch structure

*Note: The O3R platform can handle multiple data streams:*
The `camera.launch` file only launches a node for one data stream, on the default pcic port 50010. To launch a node for a different port, use:
``` 
$ roslaunch ifm3d_ros_driver camera.launch pcic_port:=<PORT_NUMBER>
```


The launch file(s) encapsulate several features:  
1. It (partially) exposes the `camera_nodelet` parameters as command-line arguments for ease of runtime configuration.
2. It instantiates a nodelet manager which the `camera_nodelet` will be loaded into.
3. It launches the camera nodelet itself.
4. It publishes the static transform from the camera's optical frame to a traditional ROS sensor frame as a tf2 `static_transform_publisher`.

You can either use [this launch file](launch/camera.launch) directly, or, use it as a basis for integrating `ifm3d_ros` into your own robot software system.

> Note: the O3R camera heads carry two imagers, a 3D time-of-flight and a RGB imager. 
> We provide the `head.launch` launchfile to handle a whole O3R camera head, that starts two nodes, one for the RGB image (we assume it is plugged in port 0), and one for the 3D imager (we assume it is plugged in port 2). 
> Other launchfiles for different configurations are available in the example package ifm3d_ros_examples.
> TODO: link to example package readme 

## Building launch files distributed systems
>Todo: add docu for how to get data into RVIZ from a ros node running on e.g. the VPU.

# LICENSE
Please see the file called [LICENSE](LICENSE).