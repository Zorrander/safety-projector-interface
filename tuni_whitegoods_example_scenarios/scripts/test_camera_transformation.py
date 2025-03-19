#!/usr/bin/env python3

import rospy
import cv2
from pathlib import Path
import time
import rospy
import actionlib

from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import Load
from geometry_msgs.msg import Point32
import moveit_commander 
from geometry_msgs.msg import Pose, Point

from tuni_whitegoods_msgs.srv import *
import math 

from integration.msg import *
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

pixel_transformation_client = rospy.ServiceProxy('pixel_2_robot', TransformPixelTo3D)

circle_centers = []
# stores mouse position in global variables ix(for x coordinate) and iy(for y coordinate) 
# on double click inside the image
def select_point(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK: # captures left button double-click
        print(x,y)
        circle_centers.append((x,y))

img = cv2.imread(str(Path.home() / 'rgb_img.png'), cv2.IMREAD_UNCHANGED)

cv2.namedWindow('image')
# bind select_point function to a window that will capture the mouse click
cv2.setMouseCallback('image', select_point)
cv2.imshow('image',img)
cv2.waitKey(0)   
cv2.destroyAllWindows()


moveit_commander.roscpp_initialize([])
rospy.init_node('pick_and_place_test')

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

rospy.sleep(5)

move_group.set_max_velocity_scaling_factor(0.5)
move_group.set_max_acceleration_scaling_factor(0.5)


def draw_circle(x, y, z=0):
    approach_z = 0.35

    if z==0:
        contact_z = 0.297
    else:
        contact_z = z 

    robot_pose = move_group.get_current_pose().pose

    new_position = Point()
    new_position.x = x
    new_position.y = y
    new_position.z = approach_z

    robot_pose.position = new_position

    move_group.set_pose_target(robot_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    robot_pose.position.z = contact_z

    move_group.set_pose_target(robot_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Circular motion parameters
    radius = 0.04
    angle_resolution = 0.05  # Radians between waypoints

    # Current pose as reference
    center_pose = move_group.get_current_pose().pose

    start_pose = move_group.get_current_pose().pose
    print(start_pose)

    # First point on circle circumference
    start_pose.position.x += radius  # - moves forward + moves back towards base

    move_group.set_pose_target(start_pose)
    move_group.go(wait=True)

    waypoints = []

    for i in range(1, int(2 * math.pi / angle_resolution) + 1):
        # Calculate waypoint coordinates
        theta = angle_resolution * i
        waypoint = Pose()
        waypoint.position.x = center_pose.position.x + radius * math.cos(theta)
        waypoint.position.y = center_pose.position.y + radius * math.sin(theta)
        waypoint.position.z = center_pose.position.z  # Keep constant Z
        waypoint.orientation = center_pose.orientation  # Maintain orientation

        waypoints.append(waypoint)

    # Compute Cartesian path
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.005, False)  

    # Execute the trajectory
    move_group.execute(plan, wait=True)

    move_group.stop()
    move_group.clear_pose_targets()

    robot_pose.position.z = approach_z

    move_group.set_pose_target(robot_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()



for center_id, center in enumerate(circle_centers):
    x = center[0]
    y = center[1]

    new_point = pixel_transformation_client(x, y, 0)

    print(new_point)
    
    if center_id in [6, 7, 10]:
        draw_circle(-new_point.x, -new_point.y, 0.295)
    else :
        draw_circle(-new_point.x, -new_point.y)
