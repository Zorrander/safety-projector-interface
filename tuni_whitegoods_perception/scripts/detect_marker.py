#!/usr/bin/env python3

import cv2
import tf2_ros as tf2
import rospy
import math
import message_filters
import numpy as np
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped, Transform
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32

class MarkerDetector(object):
    def __init__(self):
        rospy.init_node('marker_detector')
        # subscribe to the RGB image
        # Crete message filters for synchronizing the RGB and Depth top
        self.bridge = CvBridge()
        self.arucoDict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_50)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoParams.adaptiveThreshConstant = 10

        camera_name = rospy.get_param("camera_name")
        
        rgb_sub = message_filters.Subscriber("/" + camera_name + "/rgb/image_raw", Image)
        depth_sub = message_filters.Subscriber(
            "/" + camera_name + "/depth_to_rgb/image_raw", Image)

        # Use ApproximateTimeSynchronizer to sync the messages based on timestamps
        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1)
        sync.registerCallback(self.callback_image)

        self.vis_pub = rospy.Publisher(
            "visualization_marker", Marker, queue_size=10)

   # subscriber that get the RGB image
    def callback_image(self, msg, depth_msg):
        rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        self.find_dynamic_ui_transform(rgb_img, depth_image)


    # method called when the display is dynamic like on a moving table.
    # it has to update the transform so it always fit on the table
    def find_dynamic_ui_transform(self, rgb_img, depth_image):
        gray = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)
        #gray = cv2.equalizeHist(gray)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
            gray, self.arucoDict, parameters=self.arucoParams)
        if len(corners) == 0:
            # print("No marker detected")
            pass
        else:
            markerCorner = corners[0].reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = markerCorner
            
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            
            center_x =  (topRight[0] + bottomRight[0] + bottomLeft[0] + topLeft[0]) / 4
            center_y =  (topRight[1] + bottomRight[1] + bottomLeft[1] + topLeft[1]) / 4

            if self.has_moved(center_x, center_y) or self.previous_center_x is None or self.previous_center_y is None:
                self.zone_msg.top_left = [topLeft[0],
                                             topLeft[1]]
                self.zone_msg.top_right = [topRight[0],
                                          topRight[1]]
                self.zone_msg.bottom_right = [bottomRight[0],
                                           bottomRight[1]]
                self.zone_msg.bottom_left = [bottomLeft[0],
                                              bottomLeft[1]]

                response = self.transform_moving_table(self.zone_msg)
                matrix_msg = DynamicArea()
                matrix_msg = response.table_corners
                self.zone_pub.publish(matrix_msg)
            
            self.previous_center_x = center_x
            self.previous_center_y = center_y

if __name__ == '__main__':
    marker_detector = MarkerDetector()
    rospy.spin()
