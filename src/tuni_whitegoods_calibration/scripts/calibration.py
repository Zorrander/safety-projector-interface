#!/usr/bin/env python3

#this code calibrate the camera with the videoprojector
#The homographies serve to display the smart interface and another code find the homographies to deisplay the borders
#the aruco markers detection only detect the appropriate corner -> id 0 top left corner of the marker, id 1 top right of the marker ... id 3 bottom left of the marker
#inspired by this method https://dvic.devinci.fr/tutorial/how-to-automatically-calibrate-a-camera-projector-system

from pathlib import Path
import rospy
import numpy as np
from math import pi
import math
import cv2
from cv2 import aruco
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from unity_msgs.msg import InterfacePOI
from unity_msgs.msg import ElementUI
import yaml
import os

class Calibration(object):
   def __init__(self):
      super(Calibration, self).__init__()
       
      self.test_pub = rospy.Publisher("/test/project_aruco", Image,queue_size=10)
      self.sub_camera = rospy.Subscriber("/rgb/image_raw", Image, self.callback_image)
      self.sub_buttons = rospy.Subscriber("/interface_poi/buttons", InterfacePOI, self.callback_buttons)
      
      self.get_image = True
      self.rgb_img = None

      #resolution of projector
      self.screen = (1920, 1080)
      self.projected_coords = [[800,400], [1000, 400], [1000,550],[800,550]] #topleft topright bottomright bottom left
      self.projected_coords_pts = [[800,400], [1000, 400], [1000,550],[800,550]]
      self.pts_projected =  np.float32([self.projected_coords_pts[0],self.projected_coords_pts[1],self.projected_coords_pts[2],self.projected_coords_pts[3]])
      self.pts_screen = np.float32([[0,0],[self.screen[0],0],[self.screen[0],self.screen[1]],[0,self.screen[1]]])
     
      self.corners_table_rgb = []
      self.corners_table_rgb_rev = []
      self.detected_projected = []
      self.detected_projected_rev = []

      self.dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
      self.parameters = aruco.DetectorParameters()
      #get he folder where to save the homographies
      self.name_f = Path(rospy.get_param("calibration_folder"))
      self.folder = self.name_f.parent / "homography" 
      self.home = os.environ.get("HOME")

   #get RGB camera image
   def callback_image(self,msg):
      #if self.get_image:
      self.rgb_img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
      self.get_image = False

   #for debugging, to be sure that the homographies are good. 
   def callback_buttons(self,msg):
      for i in msg.poi:
         x = i.elem.x
         y = i.elem.y
         center = (int(x), int(y))
         cv2.circle(self.rgb_img, center, 4, (0, 0, 255), -1) 
      cv2.imshow("buttons", self.rgb_img)
      cv2.waitKey(5)

   #detect the aruco corners and sort them in order - top left, top-right, bottom-right, bottom-left
   def detect_aruco_corners(self):
      corners, ids, rejectedImgPoints = aruco.detectMarkers(self.rgb_img, self.dict_aruco, parameters=self.parameters)
      sum_arr = 0
      tmp = []
      ids = ids.flatten()
      top_left = [0,0]
      top_right = [0,0]
      bottom_right = [0,0]
      bottom_left = [0,0]
      img = self.rgb_img
      if len(ids) == 4:
         for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            if len(corners) == 4:
               (topLeft, topRight, bottomRight, bottomLeft) = corners
               if markerID == 0:
                  topLeft = (int(topLeft[0]), int(topLeft[1]))
                  top_left = [topLeft[0],topLeft[1]]
                  cv2.circle(img, topLeft, 4, (0, 0, 255), -1) #topleft red
               if markerID == 1:
                  topRight = (int(topRight[0]), int(topRight[1]))
                  top_right = [topRight[0],topRight[1]]
                  cv2.circle(img, topRight, 4, (255, 0, 0), -1) #topright blue
               if markerID == 2:
                  bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                  bottom_right = [bottomRight[0],bottomRight[1]]
                  cv2.circle(img, bottomRight, 4, (0, 255, 0), -1) #bottomright green
               if markerID == 3:
                  bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                  bottom_left = [bottomLeft[0],bottomLeft[1]]
                  cv2.circle(img, bottomLeft, 4, (255, 255, 0), -1)
               sum_arr = sum_arr + 1
      if sum_arr == 4:
         tmp = [top_left,top_right,bottom_right,bottom_left]

      return tmp, img
   
   #detect a single corner of a marker. id 0 top-left marker corner, id 1 - top-right marker corner, id 2 - bottom-right marker corner, id 3 - bottom-left marker corner
   def detect_single_corner(self):
      corners, ids, rejectedImgPoints = aruco.detectMarkers(self.rgb_img, self.dict_aruco, parameters=self.parameters)
      sum_arr = 0
      tmp = []
      ids = ids.flatten()
      top_left = [0,0]
      top_right = [0,0]
      bottom_right = [0,0]
      bottom_left = [0,0]
      img = self.rgb_img
      cor = -1
      pt = [0,0]
      for (markerCorner, markerID) in zip(corners, ids):
         # extract the marker corners (which are always returned in
         # top-left, top-right, bottom-right, and bottom-left order)
         corners = markerCorner.reshape((4, 2))
         if len(corners) == 4:
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            if markerID == 0:
               cor = markerID
               topLeft = (int(topLeft[0]), int(topLeft[1]))
               top_left = [topLeft[0],topLeft[1]]
               pt = top_left
               cv2.circle(img, topLeft, 4, (0, 0, 255), -1) #topleft red
            if markerID == 1:
               cor = markerID
               topRight = (int(topRight[0]), int(topRight[1]))
               top_right = [topRight[0],topRight[1]]
               pt = top_right
               cv2.circle(img, topRight, 4, (255, 0, 0), -1) #topright blue
            if markerID == 2:
               cor = markerID
               bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
               bottom_right = [bottomRight[0],bottomRight[1]]
               pt = bottom_right
               cv2.circle(img, bottomRight, 4, (0, 255, 0), -1) #bottomright green
            if markerID == 3:
               cor = markerID
               bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
               bottom_left = [bottomLeft[0],bottomLeft[1]]
               pt = bottom_left
               cv2.circle(img, bottomLeft, 4, (255, 255, 0), -1)
            sum_arr = sum_arr + 1
      return img, cor, pt

   #draw an image with one marker on each corner - id 0 top-left, id 1 - top-right, id 2 - bottom-right, id 3 - bottom-left
   def draw_aruco_projector(self):
      #place aruco patters on images at projected_coords coordinates
      arucoFrame=np.full((1080,1920,3), 255,np.uint8)
      aruco0 = cv2.imread("/home/odin-lms2/safety-projector-interface/src/tuni_whitegoods_calibration/arucos/aruco0.png")
      aruco1 = cv2.imread("/home/odin-lms2/safety-projector-interface/src/tuni_whitegoods_calibration/arucos/aruco1.png")
      aruco2 = cv2.imread("/home/odin-lms2/safety-projector-interface/src/tuni_whitegoods_calibration/arucos/aruco2.png")
      aruco3 = cv2.imread("/home/odin-lms2/safety-projector-interface/src/tuni_whitegoods_calibration/arucos/aruco3.png")

      arucoFrame[self.projected_coords[0][1]:self.projected_coords[0][1]+aruco0.shape[0], self.projected_coords[0][0]:self.projected_coords[0][0]+aruco0.shape[1]] = aruco0
      arucoFrame[self.projected_coords[1][1]:self.projected_coords[1][1]+aruco1.shape[0], self.projected_coords[1][0]:self.projected_coords[1][0]+aruco1.shape[1]] = aruco1
      arucoFrame[self.projected_coords[2][1]:self.projected_coords[2][1]+aruco2.shape[0], self.projected_coords[2][0]:self.projected_coords[2][0]+aruco2.shape[1]] = aruco2
      arucoFrame[self.projected_coords[3][1]:self.projected_coords[3][1]+aruco3.shape[0], self.projected_coords[3][0]:self.projected_coords[3][0]+aruco3.shape[1]] = aruco3

      return arucoFrame 
   
   #Here, we place aruco markers on the table where the interface will be displayed
   #it match the detected markers
   def calibrate_projection_to_table(self):
      print("place arucos on the corners")
      k = 0
      while k == 0:
         cv2.imshow("detected", self.rgb_img)
         k = cv2.waitKey(0)
         
      ok = False
      corners = []
      while not ok:
         corners, img = self.detect_aruco_corners()
         cv2.imshow("detected", img)
         k = cv2.waitKey(0)
         if k == 99:
            ok = True
         #cv2.imwrite("/home/altair/odin/src/calibration/homography/detected_corners_rgb.jpg",img)
      cv2.destroyWindow("detected")
      self.pts_table = np.float32([corners[0],corners[1],corners[2],corners[3]])
      self.corners_table_rgb_rev = self.pts_table

   #project some markers in the corners and detect them
   #the goal is to match the projected markers with the detected ones
   # The detection happens one by one begining by top left, so hide the other ones.
   def calibrate_detected_to_projection(self):
      ok = False
      corners = []
      tmp = []
      print("trying...press c if the detected point are satisfying")
      projected_aruco_img = self.draw_aruco_projector()
      self.test_pub.publish(CvBridge().cv2_to_imgmsg(projected_aruco_img, encoding="passthrough"))
      cv2.namedWindow("projection", cv2.WND_PROP_FULLSCREEN)
      cv2.setWindowProperty("projection", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
      #modify second parameter to the X resolution of your screen here
      cv2.moveWindow("projection", 1920, 0)
      cv2.imshow("projection", projected_aruco_img)
      cv2.waitKey(50)
      rospy.sleep(5.0)
      s = 0
      while s < 4:
         img, corner, pt = self.detect_single_corner()
         cv2.imshow("detected", img)
         k = cv2.waitKey(0)
         print(k)
         if k == 99:
            tmp.append([corner,pt])
            s += 1
         #cv2.imwrite("/home/altair/odin/src/calibration/homography/detected_projected.jpg",img)

      pts = []
      for i in range(0,4):
         for j in range(0,4):
            if tmp[j][0] == i:
               pts.append(tmp[j][1])
      self.detected_projected_rev = np.float32([pts[0],pts[1],pts[2],pts[3]])
      cv2.destroyWindow("projection")
      cv2.destroyWindow("detected")
      
   #IMPORTANT -> HIDE ALL CORNERS EXCEPT ONE FOR THE PROJECTION, THEN SHOW MARKER ONE BY ONE.
   #FIRST POINT COUNT AND WAIT "c" to record second
   #the calibration works for a single camera-videoprojection system. Repeat the process for a second one.
   def system_calibration(self):
      name_zone = input("name of the zone to calibrate : ")
      self.calibrate_detected_to_projection()
      self.calibrate_projection_to_table()
      print("pts_screen",self.pts_screen)
      print("pts_projected",self.pts_projected)
      print("detected_projected_rev",self.detected_projected_rev)
      print("corners_rev",self.corners_table_rgb_rev)
      #get the matrix transofmr that would be the homopraphies
      
      m1 = cv2.getPerspectiveTransform(self.pts_screen,self.pts_projected)
      # Transformation between the corners detected in the camera coordinates frame and the points as projected by the projector.
      m2 = cv2.getPerspectiveTransform(self.detected_projected_rev,self.pts_projected)
      # Transformation between the corners as projected by the projector and the ones manually place
      m3 = cv2.getPerspectiveTransform(self.pts_projected,self.corners_table_rgb_rev)
      projection_matrix = m1.dot(m2).dot(m3)
      projection_matrix_moving = m1.dot(m2)
      name_hom_proj_static = self.home + "/hom_proj_static.npy"
      name_hom_proj_moving = self.home + "/hom_proj_moving.npy"
      name_cam_screen = self.home + "/hom_cam_screen_to_proj.npy"
      name_hom_cam_static = self.home + "/hom_cam_static.npy"
      np.save(name_hom_proj_static,projection_matrix)
      np.save(name_hom_proj_moving,projection_matrix_moving)
      np.save(name_cam_screen,m1)
      np.save(name_hom_cam_static,m3)
      #generate calbrations
      n_hps = "hom_proj_static"
      n_hpm = "hom_proj_moving"
      n_hcsp = "hom_cam_screen_to_proj"
      n_hcs = "hom_cam_static"
      #self.save_matrix("hom_proj_static",projection_matrix) #for projection on static surface
      #self.save_matrix("hom_proj_moving",projection_matrix_moving) #for projection on moving surface
      #self.save_matrix("hom_cam_screen_to_proj",m1) #for projection of interface to projection surface - for detection
      #self.save_matrix("hom_cam_static",m3) #transform to static surface - for detection

      cam = {
         'id':0,
         'zone':name_zone,
         n_hcsp:name_cam_screen,
         n_hcs:name_hom_cam_static
      }
      proj1 = {
         'id':0,
         'zone':name_zone,
         n_hps:name_hom_proj_static,
         n_hpm:name_hom_proj_moving
      }
      data = {
         'cam' : [cam],
         'proj': [proj1], 
      }
      #write info
      file_yml = self.home+"/projection_calibration.yaml"
      with open(file_yml, 'w') as file:
         documents = yaml.dump(data, file)

      print("end script")
      
   #not used, for debugging
   def test(self):
      from geometry_msgs.msg import Pose 
      from std_msgs.msg import ColorRGBA
      from integration.msg import VirtualButtonReference

      pub_button = rospy.Publisher("/interfaceUI/openflow/new_button", VirtualButtonReference, queue_size=10)

      test_button = VirtualButtonReference()
      test_button.zone = 'table'
      test_button.name = 'test button'
      test_button.description = 'A button to test calibration'
      test_button.text = 'test'
      test_button.button_color = ColorRGBA(1.0, 0.0, 0.0, 0.0)
      test_button.text_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
      test_button.center = Pose()
      test_button.center.position.x = 500
      test_button.center.position.y = 200
      test_button.radius = 70 
      print(test_button)
      pub_button.publish(test_button)


if __name__ == '__main__':
   rospy.init_node('calibration') 
   cal = Calibration()
   rospy.sleep(2.0)
   #cal.system_calibration()
   cal.test()
   rospy.spin()