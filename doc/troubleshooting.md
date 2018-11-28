ifm3d-ros Troubleshooting Guide
=============================

You can use this guide to help you identify and resolve problems you may
experience in using the ifm3d-ros package.

# List of contents:

- [ifm3d-ros services provide no response.](#ifm3d-ros-services-provide-no-response)

## ifm3d-ros services provide no response
On systems utilising a single core processor,
you may find that the Dump, Config and Trigger services of ifm3d-ros package do
not provide any response when invoked.

This issue can be resolved by setting the <b>"num_worker_threads"</b> parameter
for your ROS nodelet manager to use a value > 1. You can read more about this
parameter [here](http://wiki.ros.org/nodelet).

The snippet below show's how to set this parameter using the
[nodelet.launch](https://github.com/ifm/ifm3d-ros/blob/master/launch/nodelet.launch)
file of the ifm3d-ros package.

```
  <node pkg="nodelet"
        type="nodelet"
        name="$(arg camera)_standalone_nodelet"
        args="manager"
        output="screen">
      <param name="num_worker_threads" value="2" />
  </node>
```

Alternatively if a virtual machine is being used, configuring it to utilise
more than one core should resolve this issue without any changes to the launch
file.
