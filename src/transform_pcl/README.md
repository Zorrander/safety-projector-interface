# Package Transform PCL

Transform a pointcloud from camera frame to robot frame. The transformed is passed through ROS parameters.
First the voxel transform has to be launched to reduce the size of pointclouds (tuni_whitegoods voxel.launch). More info (https://wiki.ros.org/pcl_ros/Tutorials/VoxelGrid%20filtering)
The config/icp.txt is the transformation matrix used by the slave camera to align it with the master.
Basically, the package needs to be executed on each computer with a connected camera. Then, it has to be decided which camera will be the master and which will be the slave.
Then, the ICP transform each slave (if more than 2 cameras) to align with the master frame.

## Careful

It all depends on the robot camera calibration, and from which camera frame the PCL comes from. The whitegoods environment has a robot calibration from rgb_frame to robot base. The HRC has a calibration from depth frame to robot frame.
This has a great importance for detection of interactions in the package depth_interface.

## Process

For master camera, the code transform pcl(depth_frame) -> robot frame
For slave camera, the code transform pcl(depth_frame) -> robot frame -> icp transform

For the icp transform, it is necessary to find the parameters through the icp calibration wizard 

The whitegoods environment has a robot calibration from rgb_frame to robot base. The HRC has a calibration from depth frame to robot frame.

## One camera system
These parameters are necessary to run one instance of the transform node.

For a system with one camera (like whitegoods), start the voxel master and the transform :
```
roslaunch tuni_whitegoods voxel.launch
roslaunch tuni_whitegoods transform.launch
```
The parameters necessary in transform.launch are :
```
<param name="~robot_base" value="base" />
```
Name of the robot base frame. It will be used for the pointcloud.
```
<param name="~name_transform_robot_space" value="master_to_base" />
```
Name of the transform from this camera to the robot base
```
<param name="~sub_points" value="/voxel_grid_master/output" />
```
Topic name of the voxel filter. This is where we are going to receive the pointcloud.
```
<param name="~topic_pcl_master" value="/kinect_master/cloud_robot_frame" />
```
Topic name of the pointcloud transformed generated

Params are private because several instances of the same code can run (multiple camera), it avoid to hardcode parameters inside the code.
### 2 or more cameras
For more than one camera, the ros package will be executed on each of the machines connected to a camera. Example here with 2 cameras (to adapt with the package name) :

Master camera has the same parameters as previous. For the slave camera () :

```
roslaunch tuni_whitegoods voxel.launch     
roslaunch tuni_whitegoods transform.launch
```
The parameters necessary in transform.launch are :
```
<param name="robot_base" value="base_slave" />
```
Name of the robot base frame.
```
<param name="~name_transform_robot_space" value="master_to_base" />
```
Name of the ros param containing the specific transform from this camera to the robot frame
```
<param name="~sub_points" value="/voxel_grid_sub/output" />
```
Topic name of the voxel filter. This is where we are going to receive the pointcloud.
```
<param name="~topic_pcl_sub" value="/kinect_sub/cloud_robot_frame" />
```
Topic name of the pointcloud transformed
