# ODIN PROJECT

More details on the packages git pages.
The icp package is a c++ file using pcl to perform icp and find the right rotation/translation matrix.

## projector_opt

C++ Package that display interface on the projectors.

## transform_pcl

C++ Package that transforms point clouds from both kinects and align them to the robot frame.

## pcl_fusion

C++ Package that merges both point clouds from pcl_transform and generate a depth map

## How to launch :

roslaunch pcl_transform voxel_and_pcltf.launch

roslaunch pcl_fusion fuse.launch


## CURENT ISSUES

The calibration of the homography does not work. The interface projected is ahead of 1m from the table.
The image seems parallel to  the table though.

The filtering, transform of the pointclouds to robot frame, fusion of the pointclouds and generation of the depthmap 
is quite efficient. However, the process is slow because the kinects publish a pcl only every 200ms.
After investigation, it seems that a single computer can't handle 2 kinects (BUS bottleneck) but more investigation and tweaking are necesary.

The depthmap is on format CV_8UC3 which is good for display but not optimal for depth detection. I'm figuring out a way of using CV_16FC1

