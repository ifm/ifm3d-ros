# merging point clouds with `merge_pc.launch`

Three parts:  
1. one node per imager publishing data under it's own namespace
2. set all 3D imagers to run mode using a rosservice call to `SoftOn`
3. link ifm3d optical frame to map frame via a dummy transform publisher: `tf2_ros` with pose parameters = 0