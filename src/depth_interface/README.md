# Depth Interface Package

This package transform RGB coordinates to their location in the depthmap. It is fundamental for the hand detection and the button interface detection.

The hand tracker send RGB cordinates here. This package delivers these coordinates in the depthmap.

The projector package sends the button coordinates here. The package gives the buttons coordinates in the depthmap.

# Parameters

The necessary parameters are the following :
```
<param name="~robot_base" value="base" />
```
Name of the robot base
```
<param name="~topic_depth" value="/depth/image_raw" />
```
Topic of the depth camera
```
<param name="~topic_depth_to_rgb" value="/depth_to_rgb/image_raw" />
```
Topic of the depth image transform to RGB frame. This operation is done by the Azure ROS framework.
```
<param name="~name_transform_robot_space" value="master_to_base" />
```
Name of the param that provide the camera robot transform. This transform should be available in the global namespace in the ros_params.launch

# Launch

an example Launch is present in whitegoods :
```
roslaunch tuni_whitegoods depth_interface.launch
```