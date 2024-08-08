#!/usr/bin/env python3


import cv2
import math
import numpy as np
from math import pi
from cv2 import aruco
import mediapipe as mp


import rospy
from tuni_whitegoods_msgs.msg import HandsState
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError


class HandTracker(object):
   def __init__(self, mode=False, maxHands=2, detectionCon=0.9, modelComplexity=1, trackCon=0.9):
      rospy.init_node('hand_tracking')
      #subscribe to the RGB image
      self.sub_camera = rospy.Subscriber("/rgb/image_raw", Image, self.callback_image)
      self.bridge = CvBridge()

      #publish the RGN coordinates
      self.pub_hands_poi = rospy.Publisher("/hand_tracking/rgb/coordinates", HandsState, queue_size=10)
      self.hand_detection_pub = rospy.Publisher("/odin/visualization/hand_detection", Image, queue_size=1)
      self.mode = mode
      self.maxHands = maxHands
      self.detectionCon = detectionCon
      self.modelComplex = modelComplexity
      self.trackCon = trackCon
      self.mpHands = mp.solutions.hands
      self.hands = self.mpHands.Hands(self.mode, self.maxHands,self.modelComplex, self.detectionCon, self.trackCon)
      self.mpDraw = mp.solutions.drawing_utils
      self.colors = [(255,0,255), (255,255,0)]
   
   #subscriber that get the RGB image
   def callback_image(self, msg):
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
   
   #get the positions of the hands. More particularly of the tip of the middle finger (id=12).
   def positionFinder(self,cv_img, hands, draw=True):
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
                     cv2.circle(cv_img,(cx,cy), 15 , (255,0,0), cv2.FILLED)
                     msg_hands.name.append("right")
                  if handType=='Left':
                        cv2.circle(cv_img,(cx,cy), 15 , (0,0,255), cv2.FILLED)
                        msg_hands.name.append("left")
                  tmp_pos = Point()
                  tmp_pos.x = cx
                  tmp_pos.y = cy
                  msg_hands.position.append(tmp_pos)
            self.pub_hands_poi.publish(msg_hands)
      #publish RGB coordinates of the hand
      self.hand_detection_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "passthrough"))
      

      return lmlist

   '''

    void DepthInterface::handTrackerCallback(const unity_msgs::poiPCLConstPtr& msg)
    {
      list_points_hand.pts.clear();

        cv::Mat cv_viz_depth_hands = cv_viz_depth.clone();
        for(int i = 0; i < msg->pts.size(); i++)
        {
          geometry_msgs::Point current_elem;
          current_elem.x = msg->pts[i].x;
          current_elem.y = msg->pts[i].y;
          list_points_hand.pts.push_back(current_elem);

          cv::circle(cv_viz_depth_hands, cv::Point(msg->pts[i].x, msg->pts[i].y), 10, cv::Scalar(255, 0, 0), 2);
          sensor_msgs::ImagePtr viz_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_viz_depth_hands).toImageMsg();
          pub_full_scene.publish(viz_msg);

          // Get the depth value at the RGB point
          // uint16_t depth_value = cv_depth_rgb.at<uint16_t>(msg->pts[i].x, msg->pts[i].y);

          // Assuming depth image is in millimeters, convert to meters
          // float depth_in_meters = depth_value / 1000.0;

        //publish hand coordinates in the depthmap
        pub_hand_tracker_dm.publish(list_points_hand);

    }

   '''

if __name__ == '__main__':
   ht = HandTracker()
   rospy.spin()