# Projector Interface

This package project everything that needs to be projected (borders, interface etc...)
For interface creation, it basically receives information from the openflow server. Otherwise, it receives the elements to be projected (static borders, dynamic borders) as image and the code associate a transform to display them correctly.

# Launch

For whitegoods

```
roslaunch projector projection_ur5.launch
```

For HRC, there need to be some harmonization in the organization of the calibration folder. The old code still works :

```
<launch>
    <node name="projection_system" pkg="projector" type="projector_interface.py" output="screen"/>
</launch>
```
