#!/usr/bin/env python3

import cv2
import rospy

import message_filters
import numpy as np
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tuni_whitegoods_msgs.srv import TransformRobotCameraCoordinates
from tuni_whitegoods_msgs.srv import TransformPixelTo3D
from tuni_whitegoods_msgs.srv import TransformMovingTable
from tuni_whitegoods_msgs.msg import DynamicArea
from geometry_msgs.msg import PoseStamped, Transform
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

class TableTracker(object):
    def __init__(self):
        rospy.init_node('moving_table_tracking')
        # subscribe to the RGB image
        # Create message filters for synchronizing the RGB and Depth topics
        rgb_sub = message_filters.Subscriber("/rgb/image_raw", Image)
        depth_sub = message_filters.Subscriber(
            "/depth_to_rgb/image_raw", Image)

        # Use ApproximateTimeSynchronizer to sync the messages based on timestamps
        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1)
        sync.registerCallback(self.callback_image)

        self.bridge = CvBridge()
        self.arucoDict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_250)
        self.arucoParams = cv2.aruco.DetectorParameters()

        self.use_moving_table = rospy.get_param("is_moving")
        self.vis_pub = rospy.Publisher(
            "visualization_marker", Marker, queue_size=10)

        self.zone_msg = DynamicArea()
        self.zone_pub = rospy.Publisher(
            "/odin/projector_interface/moving_table/transform", DynamicArea, queue_size=10)

        self.transform_moving_table = rospy.ServiceProxy(
            'transform_table_server', TransformMovingTable)

        self.transform_world_coordinates = rospy.ServiceProxy(
            'transform_world_coordinates_frame', TransformRobotCameraCoordinates)
        self.project_pixel_to_3D = rospy.ServiceProxy(
            'transform_pixel_to_3D', TransformPixelTo3D)
        self.in_point_stamped = PoseStamped()
        self.in_point_stamped.header.frame_id = "rgb_camera_link"

   # subscriber that get the RGB image
    def callback_image(self, msg, depth_msg):
        rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        if self.use_moving_table:
            self.find_dynamic_ui_transform(rgb_img, depth_image)

    def create_input_message(self, coordinates):
        self.in_point_stamped.header.stamp = rospy.Time(0)
        self.in_point_stamped.pose.position.x = coordinates.x
        self.in_point_stamped.pose.position.y = coordinates.y
        self.in_point_stamped.pose.position.z = coordinates.z

    # method called when the display is dynamic like on a moving table.
    # it has to update the transform so it always fit on the table
    def find_dynamic_ui_transform(self, rgb_img, depth_image):
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
            rgb_img, self.arucoDict, parameters=self.arucoParams)
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

            self.zone_msg.top_left = [topLeft[0],
                                         topLeft[1]]
            self.zone_msg.top_right = [topRight[0],
                                      topRight[1]]
            self.zone_msg.bottom_right = [bottomRight[0],
                                       bottomRight[1]]
            self.zone_msg.bottom_left = [bottomLeft[0],
                                          bottomLeft[1]]

            transformation = self.transform_moving_table(self.zone_msg)
            matrix_msg = DynamicArea()
            matrix_msg = transformation.table_corners
            self.zone_pub.publish(matrix_msg)


if __name__ == '__main__':
    table_tracker = TableTracker()
    rospy.spin()
