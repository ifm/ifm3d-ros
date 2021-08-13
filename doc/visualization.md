## HOW to visualize the point cloud with RVIZ
The included launch file `camera.launch` will publish and remap all topics and services to `/ifm3d/xxx`, for example the point cloud topic will be remapped to `/ifm3d/camera/cloud`.  

When you open RVIZ for the first time and subscribe the point cloud topic to it, it will not be displayed as the transformation chain between the different reference frames is not complete. We suggest using "dummy" a static transform publisher to fix the missing link in the pose transformation chain:  

Open a new shell and run this simple `static_transform_publisher` to map the `ifm3d/camera_link` to the `map` frame.
```
rosrun tf2_ros static_transform_publisher 1 0 0 0 0 0 map ifm3d/camera_link
```

