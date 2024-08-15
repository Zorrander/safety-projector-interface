#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point 
from tuni_whitegoods_msgs.srv import TransformRobotCameraCoordinates
from tuni_whitegoods_msgs.srv import TransformPixelTo3D
from tuni_whitegoods_msgs.msg import DynamicArea
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

        self.use_moving_table = rospy.get_param("is_moving")
        self.vis_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10);

        self.zone_msg = DynamicArea()
        self.zone_pub = rospy.Publisher("/odin/projector_interface/moving_table", DynamicArea, queue_size=10)

        self.transform_world_coordinates = rospy.ServiceProxy('transform_world_coordinates_frame', TransformRobotCameraCoordinates)
        self.project_pixel_to_3D = rospy.ServiceProxy('transform_pixel_to_3D', TransformPixelTo3D)
        self.in_point_stamped = PoseStamped()
        self.in_point_stamped.header.frame_id = "rgb_camera_link"

   #subscriber that get the RGB image
    def callback_image(self, msg):
        rgb_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        if self.use_moving_table:
        	self.find_dynamic_ui_transform(rgb_img)

    def create_input_message(self, coordinates):
	    self.in_point_stamped.header.stamp = rospy.Time(0)
	    self.in_point_stamped.pose.position.x = coordinates.x
	    self.in_point_stamped.pose.position.y = coordinates.y
	    self.in_point_stamped.pose.position.z = coordinates.z

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

	        width = abs(bottomLeft[0] - topLeft[0])
	        height = abs(topLeft[1] - topRight[1])

	        print("width: ", width)
	        print("height: ", height)
	        scale_long = width*3.85
	        scale_short = height*2.72

	        # TODO: need to add cases to infer table orientation
	        x1 = topLeft[0] 
	        x2 = topLeft[0] - int(scale_short)
	        y1 = topLeft[1] 
	        y2 = topLeft[1] + int(scale_long) if (topLeft[1] + int(scale_long)) > 0 else 0
	        	
	        print("x1: ", x1)
	        print("x2: ", x2)
	        print("y1: ", y1)
	        print("y2: ", y2)

	        tr_point_t = (x2, y1)
	        br_point_t = (x2, y2)
	        bl_point_t = (x1, y2) 

	        tl_3D_coordinates =  self.project_pixel_to_3D(topLeft[0], topLeft[1])
	        tr_3D_coordinates =  self.project_pixel_to_3D(tr_point_t[0], tr_point_t[1])
	        br_3D_coordinates =  self.project_pixel_to_3D(br_point_t[0], br_point_t[1])
	        bl_3D_coordinates =  self.project_pixel_to_3D(bl_point_t[0], bl_point_t[1])

	        # Create the input pose stamped message
	        self.create_input_message(tl_3D_coordinates) 
	        # Perform the transformation
	        top_left_robot_coordinates = self.transform_world_coordinates(self.in_point_stamped, "base")

	        self.create_input_message(tr_3D_coordinates) 
	        top_right_robot_coordinates = self.transform_world_coordinates(self.in_point_stamped, "base")

	        self.create_input_message(br_3D_coordinates)
	        bottom_right_robot_coordinates = self.transform_world_coordinates(self.in_point_stamped, "base")

	        self.create_input_message(bl_3D_coordinates)
	        bottom_left_robot_coordinates = self.transform_world_coordinates(self.in_point_stamped, "base")

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

	        self.vis_pub.publish( marker )
	        self.zone_msg.top_left     = [top_left_robot_coordinates.out_point_stamped.pose.position.x, top_left_robot_coordinates.out_point_stamped.pose.position.y]
	        self.zone_msg.top_right    = [top_right_robot_coordinates.out_point_stamped.pose.position.x, top_right_robot_coordinates.out_point_stamped.pose.position.y]
	        self.zone_msg.bottom_right = [bottom_right_robot_coordinates.out_point_stamped.pose.position.x, bottom_right_robot_coordinates.out_point_stamped.pose.position.y]
	        self.zone_msg.bottom_left  = [bottom_left_robot_coordinates.out_point_stamped.pose.position.x, bottom_left_robot_coordinates.out_point_stamped.pose.position.y]

	        self.zone_pub.publish(self.zone_msg)

if __name__ == '__main__':
	table_tracker = TableTracker()
	rospy.spin()   
