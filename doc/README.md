# General overflow

![overflow](https://github.com/rouzinho/odin/assets/10597250/25bec1e9-1c68-458c-8f08-08aab408d6cf)

Each package has a particular role in the general overflow. We will detail here which does what. For more details, visit directly the package repository.

Align point clouds -> transform_pcl

Generate scene depth map -> pcl_fusion


# ROS Params Overview

Inside the config folder for a specific app (whitegoods for example), there is the parameters.yaml file that load a set of ros params :
```
calibration_homography: /home/altair/odin/src/tuni_whitegoods/calibration/homography/
```
repository containing homography files for projection
```
file_recorded: /home/altair/odin/src/tuni_whitegoods/calibration/ur5.mkv
```
video sample of the kinect. Necessary for the detection of interaction as it uses camera intrinsic parameters encoded inside the file.
```
calibration_folder: /home/altair/odin/src/tuni_whitegoods/calibration/depthmap/
```
calibration folder of the depthmap containing various param. extreme_values.txt indicates the boundaries points of a PCL from the camera. Params.txt are the parameters that uses extreme_values.txt for the linear transform projecting the PCL points to an image.
```
is_moving: false
```
if the table displaying the interface can be moved. If false that means there is no need to continuously monitor the aruco on the table and calculate the projection transform.

```
sub_pcl_master: /kinect_master/cloud_robot_frame
```
name of the topic that output the transformed PCL of the master camera aligned with the robot frame.
```
sub_pcl_sub: /kinect_sub/cloud_robot_frame 
```
name of the topic that output the transformed PCL of the slave camera aligned with the robot frame.
```
master_to_base: [0.501192, -0.271756, 1.311792, -0.685276, 0.727083, -0.014888, 0.039050]
```
Matrix that transform a PCL from the master camera to the robot base.
