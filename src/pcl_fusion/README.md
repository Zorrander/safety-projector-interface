# Point Cloud Fusion

This package is in charge of merging pointclouds coming from several cameras together in order to generate a depthmap. The package also works with only one pointcloud.

# Launch parameters

The package needs to access the following parameters defined as global params inside the ros_params.launch :
```
number_cam
```
The number of cameras used. If there is more than 1, the code will perform an icp to match the master camera before generating the depthmap.
```
calibration_folder
```
calibration folder containings the files necessary to the creation of the depthmap. If the folder is empty, the code will generate the files at that location during the first startup of the code.
```
sub_pcl_master
```
topic name outputing the transformed pointcloud (to robot frame) coming from master camera.
```
sub_pcl_sub
```
topic name outputing the transformed pointcloud (to robot frame) coming from slave camera.

for TUNI whitegoods, the package can be lanched by :
```
roslaunch tuni_whitegoods depthmap.launch
```

# Output

The package will advertise a depthmap image on the topic /detection/depthmap. This will be the basis for any future detection.
