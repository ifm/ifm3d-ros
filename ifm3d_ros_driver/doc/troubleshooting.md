# ifm3d-ros Troubleshooting Guide

You can use this guide to help you identify and resolve problems you may experience in using the ifm3d-ros package.

# List of contents:

- [ifm3d-ros services provide no response.](#ifm3d-ros-services-provide-no-response)
- [ifm3d-ros nodelet can not connect to O3R camera head](#ifm3d-ros-nodelet-can-not-connect-to-o3r-camera-head)
- [RGB images not shown in RViz](#rgb-images-not-shown-in-rviz)

## ifm3d-ros services provide no response
On systems utilizing a single core processor, you may find that the Dump, Config and Trigger services of ifm3d-ros package do not provide any response when invoked.

This issue can be resolved by setting the <b>"num_worker_threads"</b> parameter for your ROS nodelet manager to use a value > 1. You can read more about this parameter [here](http://wiki.ros.org/nodelet).

The snippet below show's how to set this parameter using the [nodelet.launch](https://github.com/ifm/ifm3d-ros/blob/master/launch/nodelet.launch) file of the ifm3d-ros package.

```
  <node pkg="nodelet"
        type="nodelet"
        name="$(arg camera)_standalone_nodelet"
        args="manager"
        output="screen">
      <param name="num_worker_threads" value="2" />
  </node>
```

Alternatively if a virtual machine is being used, configuring it to utilize more than one core should resolve this issue without any changes to the launch file.

## ifm3d-ros nodelet can not connect to O3R camera head
If the user forgets to set the PCIC port (our standard communication port for data broadcasting) the default PCIC port argument will be used `default_pcic_port = 50010`. This TCP-IP port (`50010`) corresponds with the physical `port 0` on the VPU. The 2D RGB imager or 3D ToF imager of choice therefore needs to be connected to exactly this port.  

When these two port arguments don't match you will likely see something along these lines, being repeatedly posted to your shell:    
```
[ INFO] [1628860557.703261704]: Running dtors...
[ INFO] [1628860557.704592043]: Initializing camera...
[ INFO] [1628860558.705489893]: Initializing framegrabber...
[ INFO] [1628860558.706352595]: Initializing image buffer...
[ WARN] [1628860559.207288723]: Timeout waiting for camera!
[ WARN] [1628860559.708197339]: Timeout waiting for camera!
[ WARN] [1628860560.209029697]: Timeout waiting for camera!
[ WARN] [1628860560.709553855]: Timeout waiting for camera!
[ WARN] [1628860561.210310005]: Timeout waiting for camera!
[ WARN] [1628860561.710739497]: Timeout waiting for camera!
[ WARN] [1628860562.211714793]: Timeout waiting for camera!
[ WARN] [1628860562.712752207]: Timeout waiting for camera!
[ WARN] [1628860563.213765790]: Timeout waiting for camera!
[ WARN] [1628860563.714424040]: Timeout waiting for camera!
[ WARN] [1628860563.714658175]: Attempting to restart framegrabber...
```

The fix for this is easy: just remember the first 4 digits of the PCIC port argument will stay the same, the last one corresponds to the physical port as marked on the VPU.

## RGB images not shown in RViz
If the rgb images are send (validate with `rostopic hz <topic_name>`) but RViz is unable to visualize them, you may be missing the needed `image_transport` plugins.
The RViz error message will notify you about the missing plugin.

As `image_transport` is defined as dependency in the `package.xml` of `ifm3d_ros_driver`, it can easily be install with `rosdep install`.
If you prefer using `apt` directly, run `sudo apt install ros-noetic-image-transport`.
