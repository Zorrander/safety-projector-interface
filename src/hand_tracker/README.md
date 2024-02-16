# Hand Tracker Package

This package is in charge of tracking the hand(s) location in the RGB space. The purpose is to have a supplementary method od detecting the operator in the whitgoods case.
Tracking the hands add more resilience when the operator and the robot are working together with static borders. The robot can cross some borders while the operator is far away.
In that case, we do not want the system to stop because of the robot.

This package has to be used with the depth_interface package. The hand tracker send the RGB location of the hands and the depth_interface package transform these coordinates to the depthmap.
Then, it is possible to use depth information + hand location in the depthmap to monitor the crossing of the static borders.

# Launch
```
roslaunch hand_tracker hand_tracking.launch
```