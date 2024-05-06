#!/usr/bin/env python3

import rospy
import numpy as np
from math import pi
import math
import cv2
from cv2 import aruco
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import mediapipe as mp
from unity_msgs.msg import poiPCL
from geometry_msgs.msg import Point

#The mediapipe module does all the work


class HandTracker(object):
   def __init__(self,mode=False, maxHands=2, detectionCon=0.1,modelComplexity=1,trackCon=0.1):
      super(HandTracker, self).__init__()
      rospy.init_node('hand_tracking')
      #subscribe to the RGB image
      self.sub_camera = rospy.Subscriber("/rgb/image_raw", Image, self.callback_image)
      #publish the RGN coordinates
      self.pub_hands_poi = rospy.Publisher("/hand_tracking/rgb/coordinates", poiPCL, queue_size=1)
      self.mode = mode
      self.maxHands = maxHands
      self.detectionCon = detectionCon
      self.modelComplex = modelComplexity
      self.trackCon = trackCon
      self.mpHands = mp.solutions.hands
      self.hands = self.mpHands.Hands(self.mode, self.maxHands,self.modelComplex,
                                        self.detectionCon, self.trackCon)
      self.mpDraw = mp.solutions.drawing_utils
   
   #subscriber that get the RGB image
   def callback_image(self,msg):
      #if self.get_image:
      rgb_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
      image, nb_hand = self.handsFinder(rgb_img)
      lmList = self.positionFinder(image, nb_hand)
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
          
      #if self.results.multi_hand_landmarks:
      #   for handLms in self.results.multi_hand_landmarks:
      #         if draw:
      #            self.mpDraw.draw_landmarks(cv_img, handLms, self.mpHands.HAND_CONNECTIONS)
      return cv_img, nb_hand
   
   #get the positions of the hands. More particularly of the tip of the middle finger.
   def positionFinder(self,cv_img, hands, draw=True):
      lmlist = []
      msg_hands = poiPCL()
      if self.results.multi_hand_landmarks:
         for i in range(0,hands):
            Hand = self.results.multi_hand_landmarks[i]
            for id, lm in enumerate(Hand.landmark):
               h,w,c = cv_img.shape
               cx,cy = int(lm.x*w), int(lm.y*h)
               lmlist.append([id,cx,cy])
               if draw and id == 12:
                     cv2.circle(cv_img,(cx,cy), 15 , (255,0,255), cv2.FILLED)
                     tmp_pos = Point()
                     tmp_pos.x = cx
                     tmp_pos.y = cy
                     msg_hands.pts.append(tmp_pos)
      #publish RGB coordinates of the hand
      self.pub_hands_poi.publish(msg_hands)

      return lmlist

if __name__ == '__main__':
   ht = HandTracker()
   rospy.spin()