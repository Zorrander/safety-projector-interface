# ODIN PROJECT

# General Flow

# Prerequisites
Get the packages and follow installation procedures :

https://github.com/ros-industrial/abb_experimental.git #only keep abb_irb4600_support

https://github.com/ros-industrial/abb_egm_rws_managers

https://github.com/ros-industrial/abb_libegm

https://github.com/ros-industrial/abb_librws

https://github.com/ros-industrial/abb_robot_driver

Install the Azure kinect SDK :
```
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install cmake
sudo apt-get install libgtk2.0-dev
sudo apt-get install git
sudo apt-get install ffmpeg
sudo apt-get install libusb-1.0
```

As the SDK is not officially supported for Ubuntu 20.04, there is a hack :

```
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
curl -sSL https://packages.microsoft.com/config/ubuntu/18.04/prod.list | sudo tee /etc/apt/sources.list.d/microsoft-prod.list
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-get update
sudo apt install libk4a1.3-dev
sudo apt install libk4abt1.0-dev
sudo apt install k4a-tools=1.3.0
```
Then you can hold the package k4a-tools in your package manager so it won't ask for update any time you install something.
Check your installation there :

https://github.com/microsoft/Azure-Kinect-Sensor-SDK

and don't forget :
```
sudo cp 99-k4a.rules /etc/udev/rules.d/
```
Then install the Azure Kinect ROS Drivers :
```
https://github.com/microsoft/Azure_Kinect_ROS_Driver
```

# Starting a calibrated environment

## TUNI whitegoods

Have a sample of the camera for calibration :
```
k4arecorder -d WFOV_UNBINNED -c 720p -r 15 -l 5 --imu OFF output.mkv
```
Load parameters (modify the calibration folders to fit your environment) :

```
roslaunch tuni_whitegoods ros_params.launch
```

Start the kinect :

```
roslaunch tuni_whitegoods kinect_simple.launch
```

Start the depthmap monitoring :

```
roslaunch tuni_whitegoods tf_dm.launch
```

You can check in RVIZ if you can see the depthmap image in /detection/depth_map

Launch the packages that will project borders and detect interactions :

```
roslaunch tuni_whitegoods projection_laptop.launch
```

The UR5 drivers can be installed here : https://github.com/UniversalRobots/Universal_Robots_ROS_Driver . It is better to go for a distribution install and not build them from sources.

Then to start the robot : 
```
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=192.168.125.200 kinematics_config:="path_to_robot_calib.yaml"
```

### How to use

To control the UR5, we basically send commands to /ur_hardware_interface/script_command. For example sending as a string :
```
movej([-0.889868,-1.497510,-1.858239,-1.393576,1.441737,2.115242], a=0.3, v=0.13)
```
 
To open the gripper, we load the program by calling service  /ur_hardware_interface/dashboard/load_program open.urp
Then we play the program by calling service /ur_hardware_interface/dashboard/play
To close the gripper, same thing as before but the program is called cl.urp.

An example is avilable in test_interface package



## HRC

There are 2 desktops : odin1 (master camera - the one closest to exit) and odin2 (sub). To update the repo, they have to be with a specific IP to be shared with your laptop that has internet. So, best configure your laptop with the followings:
```
local LAN to work with the robot and the system
Address 192.168.125.207 NetMask 255.255.255.0 Gateway -> your wifi
Internet profile
Everything automatic and make your computer as shared with others that are on the same network 
```
The profiles of the desktop are already defined. You will need to select the "shared" profile if you want the desktop to access internet to update the github. For the access :
```
ssh odin1@10.42.0.10 -> internet
ssh odin1@192.168.125.201 -> local
ssh odin2@192.168.125.202 -> local
ssh odin2@10.42.0.183 -> internet
```

To start the cameras, login to each desktop then :

```
cd odin
roslaunch hrc ros_params.launch #only on one of them, this will load general parameters
roslaunch hrc master_WFOV.launch (on odin1)
roslaunch hrc subordinate_WFOV.launch (on odin2)
```

Then start the filtering and transformation of the point clouds :

```
roslaunch hrc transform_master.launch (odin1)
roslaunch hrc transform_sub.launch (odin2)
```

Now you are ready to start the generation of depthmap on your laptop (or setup another desktop if there is one) :

```
roslaunch hrc fusion.launch
```

To start the robot :

```
roslaunch hrc robot.launch
```

Other modules are launched from the startup package, this was the previous folder to start the application in the HRC environment.


# Create a new HMI app from scratch
To create an application for a different environment, start the settings.py code :
```
python3 settings.py "name_of_your_app"
```
This will create a new package and initialize the folders that will contain the calibration files.
The folder calibration will host a video sample from the kinect. The sample contains internal kinect calibration that are necessary for the detection of interactions.
The folder calibration/depthmap contains the values necessary to generate a depthmap of the scene.
The folder calibration/homography contains the various homography files necessary to project everything (borders, zones etc...)
The folder config contains ROS params that should be launched at the very beginning. Check the doc folder to see what the parameters signify.

## LMS WHITEGOODS

Documentation on how to launch the nodes inside the package whitegoods

# Project status

| Feature | Robolab | Heavy Lab | Whirlpool |
|:---:|:---:|:---:|:---:|
| display static smart interface                    | 🟢 | 🔵 | 🔵 |  
| display dynamic smart interface                   | 🟢 | 🟢 | 🔵 |  
| detect smart interface interaction events         | 🟢 | 🔵 | 🔵 |  
| display static border                             | 🟢 | ❌ | 🟢 | 
| display dynamic border                            | 🔵 | 🟢 | ❌ |  
| robot booking static border                       | 🟢 | ❌ | 🔵 | 
| human booking static border                       | 🟢 | ❌ | 🔵 |  
| detect violation static border                    | 🟢 | ❌ | 🔵 | 
| robot releasing static border                     | 🟢 | ❌ | 🔵 |  
| human releasing static border                     | 🟢 | ❌ | 🔵 | 
| display instructions                              | ❌ | 🔵 | 🔵 | 
| robot control                                     | 🟢 | ❌ | 🔵 | 
| robot-camera calibration                          | 🟢 | 🟢 | 🟢 |  
| dual projector integration                        | ❌ | 🔵 | ❌ |
| dual camera integration                           | ❌ | 🟢 | ❌ |
| calibration isolated and integrated               | 🟢 | 🔵 | 🔵 |
| configuration allows to enable/disable features   | 🔵 | 🔵 | 🔵 |
| calibration process reproduceable                 | 🟢 | 🔵 | 🟢 |
| visualize components (table, shelf, ...)          | 🟢 | 🔵 | 🟢 |
| visualize projected elements                      | 🟢 | 🔵 | 🟢 |
| visualize detected elements (hands, objects, ...) | 🟢 | 🔵 | 🟢 |
| calibration of laser scanner position             | ❌ | 🔵 | ❌ |
| display of laser scanner status/information       | ❌ | 🔵 | ❌ |
| safety status: static display of polygon          | ❌ | 🔵 | 🔵 |
