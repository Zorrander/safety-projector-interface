#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DepthImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber("/depth/image_raw", Image, self.callback)
        self.depth_image = None

    def callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            self.save_depth_image(self.depth_image)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def save_depth_image(self, depth_image):
        cv2.imwrite('/home/odin-lms2/depthmap.png', depth_image)
        rospy.loginfo("Depth image saved")

def main():
    rospy.init_node('depth_image_saver', anonymous=True)
    dis = DepthImageSaver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()