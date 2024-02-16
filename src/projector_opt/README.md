# projector_opt package

This package project everything that needs to be projected. It receives a list of images with their transformations.
The package need to be run once per videoprojector.

# parameters

The package needs some parameters to be launched

```
<param name="~id" value="0" />
```
id of the projector. which match the cam id. id 0 goes with camera 0
```
<param name="~shiftX" value="0" />
```
shift the projection along the X axis. If the package is run on a laptop connected to a videoprojector, it will shift the display so it's not displaying on the aptop screen. X value should be the resolution of the screen.

# Launch
 To be launched on each desktop connected to a videoprojector. adapt the id.
```
<launch>
    <param name="~id" value="0" />
    <param name="~shiftX" value="0" />
    <node name="projector_optimized_one" pkg="projector_opt" type="projector_opt_node" output="screen"/>
</launch>
```

