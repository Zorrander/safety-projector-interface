#!/usr/bin/env python3


import rospy
import moveit_commander
import math
from geometry_msgs.msg import Pose

# Initialize MoveIt and ROS node
moveit_commander.roscpp_initialize([])
rospy.init_node('robot_control', anonymous=True)

# Robot setup
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"  # Replace this with your robot MoveGroup name
move_group = moveit_commander.MoveGroupCommander(group_name)

# Circular motion parameters
center_x = 0.4
center_y = 0.0
center_z = 0.3
radius = 0.05
angle_resolution = 0.1  # Radians between waypoints

# Current pose as reference
start_pose = move_group.get_current_pose().pose

waypoints = []

for theta in [angle_resolution * i for i in range(int(2 * math.pi / angle_resolution) + 1)]:
    # Calculate waypoint coordinates
    waypoint = Pose()
    waypoint.position.x = center_x + radius * math.cos(theta)
    waypoint.position.y = center_y + radius * math.sin(theta)
    waypoint.position.z = start_pose.position.z  # Keep constant Z
    waypoint.orientation = start_pose.orientation  # Maintain orientation

    waypoints.append(waypoint)

# Compute Cartesian path
(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  # Waypoint resolution: 1cm

# Execute the trajectory
if fraction == 1.0:
    move_group.execute(plan, wait=True)
else:
    rospy.logwarn("Could not compute full circular path.")

move_group.stop()
move_group.clear_pose_targets()

moveit_commander.roscpp_shutdown()