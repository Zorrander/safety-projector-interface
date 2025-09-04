#!/usr/bin/env python3


import cv2
import math
import numpy as np
from math import pi
from cv2 import aruco
import mediapipe as mp

import message_filters

import rospy
from tuni_whitegoods_msgs.msg import HandsState
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32

class HandTracker(object):
    def __init__(self, mode=False, maxHands=2, detectionCon=0.7, modelComplexity=1, trackCon=0.7):
        rospy.init_node('hand_tracking')

        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.modelComplex = modelComplexity
        self.trackCon = trackCon
        self.mpHands = mp.solutions.hands

        self.hands = self.mpHands.Hands(
            static_image_mode=self.mode,
            max_num_hands=self.maxHands,
            model_complexity=self.modelComplex,
            min_detection_confidence=self.detectionCon,
            min_tracking_confidence=self.trackCon
        )

        self.bridge = CvBridge()

        camera_name = rospy.get_param("camera_name")

        self.tracking_sub = rospy.Subscriber("/odin/object_detection/set_tracking_confidence", Int32, self.callback_tracking_confidence)
        self.detection_sub = rospy.Subscriber("/odin/object_detection/set_detection_confidence", Int32, self.callback_detection_confidence)
        self.complexity_sub = rospy.Subscriber("/odin/object_detection/set_complexity", Int32, self.callback_complexity)

        self.pub_hands_poi = rospy.Publisher("/odin/internal/hand_detection", HandsState, queue_size=5)

        # Create message filters for synchronizing the RGB and Depth topics
        self.rgb_sub = message_filters.Subscriber("/" + camera_name + "/rgb/image_rect_color", Image)
        self.depth_sub = message_filters.Subscriber(
            "/" + camera_name + "/depth_to_rgb/image", Image)

        # Use ApproximateTimeSynchronizer to sync the messages based on timestamps
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback_image)

    def callback_tracking_confidence(self, msg):
        self.trackCon = msg.data
        self.hands = self.mpHands.Hands(
            self.mode, self.maxHands, self.modelComplex, self.detectionCon, self.trackCon)

    def callback_detection_confidence(self, msg):
        self.detectionCon = msg.data 
        self.hands = self.mpHands.Hands(
            self.mode, self.maxHands, self.modelComplex, self.detectionCon, self.trackCon)

    def callback_complexity(self, msg):
        self.modelComplex = msg.data
        self.hands = self.mpHands.Hands(
            self.mode, self.maxHands, self.modelComplex, self.detectionCon, self.trackCon)

    def callback_image(self, msg, depth_msg):
        rgb_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

        self.results = self.hands.process(rgb_img)
        if self.results.multi_handedness:
            nb_hand = len(self.results.multi_handedness)
            self.positionFinder(rgb_img, nb_hand, depth_image)

    # get the positions of the hands. More particularly of the tip of the middle finger (id=12).
    def positionFinder(self, cv_img, hands, depth_image):
        msg_hands = HandsState()

        if self.results.multi_hand_landmarks:
            for i in range(hands):
                handType = self.results.multi_handedness[i].classification[0].label
                Hand = self.results.multi_hand_landmarks[i]
                h, w, _ = cv_img.shape
                for id, lm in enumerate(Hand.landmark):
                    cx = int(min(max(lm.x * w, 0), w - 1))
                    cy = int(min(max(lm.y * h, 0), h - 1))
                    cz = depth_image[cy, cx]
                    if id == 12:
                        msg_hands.name.append(handType.lower())
                        tmp_pos = Point()
                        tmp_pos.x = cx
                        tmp_pos.y = cy
                        tmp_pos.z = depth_image[cy, cx]
                        msg_hands.position.append(tmp_pos)
                        self.previous_center_x = cx
                        self.previous_center_y = cy
                        break
            self.pub_hands_poi.publish(msg_hands)


if __name__ == '__main__':
    ht = HandTracker()
    rospy.spin()
