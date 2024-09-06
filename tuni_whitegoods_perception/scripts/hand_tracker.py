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
      #subscribe to the RGB image
      self.pub_hands_poi = rospy.Publisher("/odin/internal/hand_detection", HandsState, queue_size=10)
      #publish the RGN coordinates
      self.mode = mode
      self.maxHands = maxHands
      self.detectionCon = detectionCon
      self.modelComplex = modelComplexity
      self.trackCon = trackCon
      self.mpHands = mp.solutions.hands
      self.hands = self.mpHands.Hands(self.mode, self.maxHands,self.modelComplex, self.detectionCon, self.trackCon)
      # self.mpDraw = mp.solutions.drawing_utils
      self.colors = [(255,0,255), (255,255,0)]
      self.send_interval = 2  
      self.count = 0  
      self.bridge = CvBridge()

      # Create message filters for synchronizing the RGB and Depth topics
      rgb_sub = message_filters.Subscriber("/rgb/image_raw", Image)
      depth_sub = message_filters.Subscriber("/depth_to_rgb/image_raw", Image)

      # Use ApproximateTimeSynchronizer to sync the messages based on timestamps
      sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
      sync.registerCallback(self.callback_image)

   #subscriber that get the RGB image
   def callback_image(self, msg, depth_msg):
      #if self.get_image:
      self.count += 1  # Increment the count
      if self.count % self.send_interval == 0:
        rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        image, nb_hand = self.handsFinder(rgb_img)
        lmList = self.positionFinder(image, nb_hand, depth_image)
        #cv2.imshow("Hand Tracker", image)  # Display the image with hand position
        #cv2.waitKey(1)
        self.count = 0
      #if len(lmList) != 0:
         #print(lmList)
      #cv2.imshow("Hand Tracker", image)
      #cv2.waitKey(1)  
   
   #find the hands in the image. 
   def handsFinder(self, cv_img, draw=False):
      nb_hand = 0
      imageRGB = cv2.cvtColor(cv_img,cv2.COLOR_BGR2RGB)
      self.results = self.hands.process(imageRGB)
      if self.results.multi_handedness != None:
          nb_hand = len(self.results.multi_handedness)         
      return cv_img, nb_hand
   
   #get the positions of the hands. More particularly of the tip of the middle finger (id=12).
   def positionFinder(self,cv_img, hands, depth_image, draw=True):
      lmlist = []
      msg_hands = HandsState()
      if self.results.multi_hand_landmarks:
          for i in range(0, hands):
            handType=self.results.multi_handedness[i].classification[0].label 
            Hand = self.results.multi_hand_landmarks[i]
            for id, lm in enumerate(Hand.landmark):
               h,w,c = cv_img.shape
               cx,cy = int(lm.x*w), int(lm.y*h)
               lmlist.append([id,cx,cy])
               if draw and id == 12:
                  if handType=='Right':
                     msg_hands.name.append("right")
                  if handType=='Left':
                        msg_hands.name.append("left")
                  #color = (0, 255, 0) if handType == 'Right' else (255, 0, 0)
                  #cv2.circle(cv_img, (cx, cy), 10, color, cv2.FILLED)  # Draw a circle at the fingertip
                  #cv2.putText(cv_img, f"{handType} Hand", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                  tmp_pos = Point()
                  tmp_pos.x = cx
                  tmp_pos.y = cy
                  tmp_pos.z = depth_image[cy, cx]/1000
                  msg_hands.position.append(tmp_pos)
          self.pub_hands_poi.publish(msg_hands)      

      return lmlist


if __name__ == '__main__':
   ht = HandTracker()
   rospy.spin()