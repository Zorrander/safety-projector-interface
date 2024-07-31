#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point 
from tuni_whitegoods_msgs.srv import TransformRobotCameraCoordinates
from tuni_whitegoods_msgs.srv import TransformPixelTo3D
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

class TableTracker(object):
    def __init__(self):
        rospy.init_node('moving_table_tracking')
        #subscribe to the RGB image
        self.sub_camera = rospy.Subscriber("/rgb/image_raw", Image, self.callback_image)
        self.bridge = CvBridge()
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.vis_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10);

        self.transform_world_coordinates = rospy.ServiceProxy('transform_world_coordinates_frame', TransformRobotCameraCoordinates)
        self.project_pixel_to_3D = rospy.ServiceProxy('transform_pixel_to_3D', TransformPixelTo3D)

   #subscriber that get the RGB image
    def callback_image(self, msg):
        rgb_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        self.find_dynamic_ui_transform(rgb_img)

    #method called when the display is dynamic like on a moving table.
    #it has to update the transform so it always fit on the table
    def find_dynamic_ui_transform(self, rgb_img):
        (corners, ids, rejected) = cv2.aruco.detectMarkers(rgb_img, self.arucoDict, parameters=self.arucoParams)
        if len(corners) == 0:
        	print("No marker detected")
        else:
	        markerCorner = corners[0].reshape((4, 2))
	        (topLeft, topRight, bottomRight, bottomLeft) = markerCorner

	        topRight = (int(topRight[0]), int(topRight[1]))
	        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
	        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
	        topLeft = (int(topLeft[0]), int(topLeft[1]))

	        width = topRight[0] - topLeft[0]
	        height = topRight[1] - bottomRight[1]

	        scale_long = width*3.85
	        scale_short = height*2.72

	        x1 = topLeft[0] 
	        x2 = topLeft[0] + int(scale_long)
	        y1 = topLeft[1] 
	        y2 = topLeft[1] - int(scale_short) if (topLeft[1] - int(scale_short)) > 0 else 0
	        
	        tr_point_t = (x2, y1)
	        br_point_t = (x2, y2)
	        bl_point_t = (x1, y2) 

	        tl_3D_coordinates =  self.project_pixel_to_3D(topLeft[0], topLeft[1])
	        tr_3D_coordinates =  self.project_pixel_to_3D(tr_point_t[0], tr_point_t[1])
	        bl_3D_coordinates =  self.project_pixel_to_3D(bl_point_t[0], bl_point_t[1])
	        br_3D_coordinates =  self.project_pixel_to_3D(br_point_t[0], br_point_t[1])

	        # Create the input pose stamped message
	        in_point_stamped = PoseStamped()
	        in_point_stamped.header.frame_id = "rgb_camera_link"
	        in_point_stamped.header.stamp = rospy.Time(0)
	        in_point_stamped.pose.position.x = tl_3D_coordinates.x
	        in_point_stamped.pose.position.y = tl_3D_coordinates.y
	        in_point_stamped.pose.position.z = tl_3D_coordinates.z

	        # Perform the transformation
	        top_left_robot_coordinates = self.transform_world_coordinates(in_point_stamped, "base")

	        in_point_stamped.header.stamp = rospy.Time(0)
	        in_point_stamped.pose.position.x = tr_3D_coordinates.x
	        in_point_stamped.pose.position.y = tr_3D_coordinates.y
	        in_point_stamped.pose.position.z = tr_3D_coordinates.z
	        # Perform the transformation
	        top_right_robot_coordinates = self.transform_world_coordinates(in_point_stamped, "base")

	        in_point_stamped.header.stamp = rospy.Time(0)
	        in_point_stamped.pose.position.x = br_3D_coordinates.x
	        in_point_stamped.pose.position.y = br_3D_coordinates.y
	        in_point_stamped.pose.position.z = br_3D_coordinates.z
	        # Perform the transformation
	        bottom_right_robot_coordinates = self.transform_world_coordinates(in_point_stamped, "base")

	        in_point_stamped.header.stamp = rospy.Time(0)
	        in_point_stamped.pose.position.x = bl_3D_coordinates.x
	        in_point_stamped.pose.position.y = bl_3D_coordinates.y
	        in_point_stamped.pose.position.z = bl_3D_coordinates.z
	        # Perform the transformation
	        bottom_left_robot_coordinates = self.transform_world_coordinates(in_point_stamped, "base")

	        # Create Rviz marker 
	        marker = Marker()
	        marker.header.frame_id = "base"
	        marker.header.stamp = rospy.Time(0)
	        marker.id = 1000
	        marker.type = Marker.LINE_STRIP
	        marker.action = Marker.ADD
	        marker.scale.x = 0.01
	        marker.color.r = 1.0
	        marker.color.g = 1.0
	        marker.color.b = 1.0
	        marker.color.a = 1.0
	 
	        marker.points.append(top_left_robot_coordinates.out_point_stamped.pose.position)
	        marker.points.append(top_right_robot_coordinates.out_point_stamped.pose.position)
	        marker.points.append(bottom_right_robot_coordinates.out_point_stamped.pose.position)
	        marker.points.append(bottom_left_robot_coordinates.out_point_stamped.pose.position)
	        marker.points.append(top_left_robot_coordinates.out_point_stamped.pose.position)

	        print(top_left_robot_coordinates.out_point_stamped.pose)
	        self.vis_pub.publish( marker )

if __name__ == '__main__':
	table_tracker = TableTracker()
	rospy.spin()   
