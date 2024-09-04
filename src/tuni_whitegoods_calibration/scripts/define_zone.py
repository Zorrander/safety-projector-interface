#!/usr/bin/env python3

import cv2
import rospy
from geometry_msgs.msg import PoseStamped

from tuni_whitegoods_msgs.srv import TransformRobotCameraCoordinates
from tuni_whitegoods_msgs.srv import TransformPixelTo3D



def select_zone():
	transform_world_coordinates = rospy.ServiceProxy('transform_world_coordinates_frame', TransformRobotCameraCoordinates)
	project_pixel_to_3D = rospy.ServiceProxy('transform_pixel_to_3D', TransformPixelTo3D)

	rgb_img = cv2.imread('/home/odin-lms2/rgb_img.png', cv2.IMREAD_UNCHANGED)

	roi = cv2.selectROI("Select ROI", rgb_img)
	x1, y1, w, h = roi
	x2 = x1 + w 
	y2 = y1 + h 
	print(x1, y1, w, h)

	top_left = project_pixel_to_3D(x1, y1)
	bottom_right = project_pixel_to_3D(x2, y2)
	top_right = project_pixel_to_3D(x2, y1)
	bottom_left = project_pixel_to_3D(x1, y2)

	in_point_stamped = PoseStamped()
	in_point_stamped.header.frame_id = "rgb_camera_link"
	in_point_stamped.header.stamp = rospy.Time(0)
	in_point_stamped.pose.position.x = top_left.x
	in_point_stamped.pose.position.y = top_left.y
	in_point_stamped.pose.position.z = top_left.z
	corner1 = transform_world_coordinates(in_point_stamped, "base")

	in_point_stamped.header.stamp = rospy.Time(0)
	in_point_stamped.pose.position.x = top_right.x
	in_point_stamped.pose.position.y = top_right.y
	in_point_stamped.pose.position.z = top_right.z
	corner2 = transform_world_coordinates(in_point_stamped, "base")

	in_point_stamped.header.stamp = rospy.Time(0)
	in_point_stamped.pose.position.x = bottom_right.x
	in_point_stamped.pose.position.y = bottom_right.y
	in_point_stamped.pose.position.z = bottom_right.z
	corner3 = transform_world_coordinates(in_point_stamped, "base")

	in_point_stamped.header.stamp = rospy.Time(0)
	in_point_stamped.pose.position.x = bottom_left.x
	in_point_stamped.pose.position.y = bottom_left.y
	in_point_stamped.pose.position.z = bottom_left.z
	corner4 = transform_world_coordinates(in_point_stamped, "base")

	print("Corner 1: ")
	print(corner1)

	print("Corner 2: ")
	print(corner2)

	print("Corner 3: ")
	print(corner3)

	print("Corner 4: ")
	print(corner4)

def main():
	rospy.init_node('zone_finder')
	select_zone()

if __name__ == "__main__":
	main()

