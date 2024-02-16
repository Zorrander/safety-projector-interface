# Things to do

## Camera robot calibration (optimization)

The camera calibration as done in whitegoods is not very precise, it has to be adjusted by hand. It would be nice to optimize it.
Besides, the method has not been tested in HRC. It would be good to find a method that works for both cases.

## Package HRC (configuration)

For the system to work in HRC, it relies on old startup package with no ros parameters and calibrations files not sorted. A proper package can be created (settings.py) and adjust all parameters such as parameters.yaml and projection_calibation.yaml to point
to the existing homographies. I just created a package named hrc that can starts the cameras, the transform of pointclouds and the generation of a global depthmap. Some things to adapt :

The icp transform performed by the subordinate camera is pointing to a local file. Using a private ROS param would be better. 

The depthmap is maybe a bit dark compared to the robotlab, it can be changed by dividing the z value in pcl_fusion_node by 100 instead of 1000, but by doing so all other z values needs to be changed as well.

For projection, the old startup package is working but it starts the old projector_interface.py who points to homographies that are in the same package.The best would be to use the projector_interface_ur5.py which works for the robotlab. Take the homographies files from the projector package and organize them in hrc package the same way it is done in tuni_whitegods.

## Projector Camera calibration (configuration)

The method has only been tested for whitegoods. It needs to be tested for HRC to deal with multiple camera-projection in the projection_calibration.yaml.
First would be good to adjust it by hand with the working homographies (in projector/homographies pkg). 

## Dynamic borders (optimization)

The creation of dynamic borders rest on some calibration of the abb robot to retrieve the joints location in 3D space. It will be better to use tf to retrieve their positions in robot space and thus avoid a calibration.

## Safety zone projection (new development)

There will be the need to display safety zones on the floor with the projector. A laser scanner will monitor the operator and diplay the safety zone depending on where he is.