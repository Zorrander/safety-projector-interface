# Package calibration

This package calibrate the camera-projector together.

# Camera robot calibration (To do first)

The camera robot calibration is done with the package aruco_hand_eye and aruco_ros (presents in src/). They are forks from existing packages :

https://github.com/jhu-lcsr/aruco_hand_eye
https://github.com/pal-robotics/aruco_ros/tree/noetic-devel

Check the aruco_hand_eye package for documentation.
The aruco_hand_eye is modified from original to fit kinect UR5 calibration (launch and code).



# Camera Projector calibration to display smart user interface

When started, the code will first display 4 aruco markers on a white background. It's to find the transformation between the screen projection and the detected markers.
Since the interface won't be drawn on the floor, it is necessary to get each marker one by one on the same level of the surface you want to display things.
So, when started, the code will first try to get the top-left aruco marker. If detected, hide the first marker (just put an object here will obstruct the detection) and make sure the second is visible. Then you can press "c".
Continue until getting the bottom-left marker.

![calib_table_overview](https://github.com/rouzinho/odin/assets/10597250/9d7adebb-a03a-4c65-9b20-da99557e48c4)

Overview

![calib_table_corner1](https://github.com/rouzinho/odin/assets/10597250/7fe36bbd-5e7e-412c-8f51-bd59923a53c3)

Top left marker first

![calib_table_corner4](https://github.com/rouzinho/odin/assets/10597250/5acf2aa0-d621-4649-a405-23b95c6b8602)

Top right marker second

![calib_table_corner3](https://github.com/rouzinho/odin/assets/10597250/991fb145-b442-439e-ba1a-6f0437d94767)

Bottom right marker third

![calib_table_corner2](https://github.com/rouzinho/odin/assets/10597250/3a8d2373-afa7-4641-90ee-bc5f49ef0688)

Bottom left marker last

Then, you have to place aruco markers yourself on the surface where you want your smart interface. The zone within the markers will be the size of the interface. the markers have to be facing toward you by placing aruco 0 on top left, aruco 1 on top-right, aruco2 on bottom-right and aruco 3 on bottom-left.
Press any keys to validate.

![calib_table_size_interface](https://github.com/rouzinho/odin/assets/10597250/1a94a7af-bbeb-4a07-b44c-158f4ada4553)

The markers position on the table is where the interface will be displayed


The code will then generate the homography files and a yaml to find them easily on other code.

## Launch

```
roslaunch calibration calibration.launch
```

## Camera projector calibration for border display

This calibration match the display of the borders (static or dynamic) with their position inside the depthmap. To do so, we are performing a simple calibration where a grid of markers will be displayed, then the user decide which marker to accept.
The markers have to be on the same level where the border will be displayed. For example : to draw static borders on a table, only accept markers that are displayed on the table. To draw a dynamic border on the floor, only accept detected markers on the floor.

![screenshot_hom_depth](https://github.com/rouzinho/odin/assets/10597250/9a381182-6dfe-40d9-9805-870b2a47ca33)

Example of detected markers. Here we want to draw borders on the wood surface, so we will only accept markers 31 32 33. There needs to be definitely more points than that, putting some white sheets on the surface help the detection.

## Launch

First launch the wrapper and the ROS service :
```
roslaunch scene_calibration start_wrapper.launch
roslaunch scene_calibration start_calibration.launch
```
Then the calibration :
```
roslaunch calibration hom_depthmap.launch
```


