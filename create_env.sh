#!/bin/bash
cd src
catkin_create_pkg $1 roscpp sensor_msgs std_msgs unity_msgs
cd $1
mkdir calibration
cd calibration
mkdir depthmap
mkdir homography
