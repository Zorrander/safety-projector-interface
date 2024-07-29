#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class RGBImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber("/rgb/image_raw", Image, self.callback)
        self.rgb_image = None

    def callback(self, data):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            self.save_rgb_image(self.rgb_image)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def save_rgb_image(self, rgb_image):
        cv2.imwrite('/home/odin3/rgb_img.png', rgb_image)
        rospy.loginfo("rgb_image saved")

def main():
    rospy.init_node('rgb_image_saver', anonymous=True)
    dis = RGBImageSaver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()