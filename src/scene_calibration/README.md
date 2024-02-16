# scene calibration

This package is used to calibrate the homography that fits RGB points to depthmap.

It consists of a cpp wrapper that receives RGB points and send them to a service. Then a cpp ROS service transform these rgb points to their location within the depthmap. Th transformed points are sent to the python script get_depth_proj_hom.py inside the calibration package.

# Launch

```
roslaunch scene_calibration start_wrapper.launch
roslaunch scene_calibration start_calibration.launch
```