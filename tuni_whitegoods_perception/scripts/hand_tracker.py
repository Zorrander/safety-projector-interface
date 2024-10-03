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


class HandTracker(object):
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, modelComplexity=0, trackCon=0.5):
        rospy.init_node('hand_tracking')
        self.pub_hands_poi = rospy.Publisher(
            "/odin/internal/hand_detection", HandsState, queue_size=1)

        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.modelComplex = modelComplexity
        self.trackCon = trackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(
            self.mode, self.maxHands, self.modelComplex, self.detectionCon, self.trackCon)

        self.colors = [(255, 0, 255), (255, 255, 0)]

        self.bridge = CvBridge()

        # Create message filters for synchronizing the RGB and Depth topics
        self.rgb_sub = message_filters.Subscriber("/rgb/image_raw", Image)
        self.depth_sub = message_filters.Subscriber(
            "/depth_to_rgb/image_raw", Image)

        # Use ApproximateTimeSynchronizer to sync the messages based on timestamps
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=1, slop=0.1)
        self.sync.registerCallback(self.callback_image)

    # subscriber that get the RGB image

    def callback_image(self, msg, depth_msg, draw=False):
        rgb_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        self.results = self.hands.process(rgb_img)
        if self.results.multi_handedness:
            nb_hand = len(self.results.multi_handedness)
            lmList = self.positionFinder(rgb_img, nb_hand, depth_image)
            if draw:
                # Display the image with hand position
                cv2.imshow("Hand Tracker", rgb_img)
                cv2.waitKey(1)

    # get the positions of the hands. More particularly of the tip of the middle finger (id=12).
    def positionFinder(self, cv_img, hands, depth_image, draw=False):
        lmlist = []
        msg_hands = HandsState()

        if self.results.multi_hand_landmarks:
            for i in range(hands):
                handType = self.results.multi_handedness[i].classification[0].label
                Hand = self.results.multi_hand_landmarks[i]
                h, w, _ = cv_img.shape
                for id, lm in enumerate(Hand.landmark):
                    cx, cy = int(lm.x*w), int(lm.y*h)
                    lmlist.append([id, cx, cy])
                    if id == 12:
                        msg_hands.name.append(handType.lower())
                        if draw:
                            color = (0, 255, 0) if handType == 'Right' else (
                                255, 0, 0)
                            # Draw a circle at the fingertip
                            cv2.circle(cv_img, (cx, cy), 10, color, cv2.FILLED)
                            cv2.putText(cv_img, f"{handType} Hand", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        tmp_pos = Point()
                        tmp_pos.x = cx
                        tmp_pos.y = cy
                        tmp_pos.z = depth_image[cy, cx]
                        msg_hands.position.append(tmp_pos)
            self.pub_hands_poi.publish(msg_hands)

        return lmlist


if __name__ == '__main__':
    ht = HandTracker()
    rospy.spin()
