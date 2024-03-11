#!/usr/bin/env python3

#this code calibrate the camera with the videoprojector
#The homographies serve to display the smart interface and another code find the homographies to deisplay the borders
#the aruco markers detection only detect the appropriate corner -> id 0 top left corner of the marker, id 1 top right of the marker ... id 3 bottom left of the marker
#inspired by this method https://dvic.devinci.fr/tutorial/how-to-automatically-calibrate-a-camera-projector-system

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
      rospy.init_node('calibration')  
      self.sub_camera = rospy.Subscriber("/rgb/image_raw", Image, self.callback_image)
      self.sub_buttons = rospy.Subscriber("/interface_poi/buttons", InterfacePOI, self.callback_buttons)
      self.get_image = True
      self.rgb_img = None
      #resolution of projector
      self.screen = (1920, 1080)
      self.projected_coords = [[4,4], [1810, 4], [1810,970],[4,970]] #topleft topright bottomright bottom left
      self.projected_coords_pts = [[4,4], [1916, 4], [1916,1076],[4,1076]]
      self.pts_projected =  np.float32([self.projected_coords_pts[0],self.projected_coords_pts[1],self.projected_coords_pts[2],self.projected_coords_pts[3]])
      self.pts_screen = np.float32([[0,0],[self.screen[0],0],[self.screen[0],self.screen[1]],[0,self.screen[1]]])
      self.corners_table_rgb = []
      self.corners_table_rgb_rev = []
      self.detected_projected = []
      self.detected_projected_rev = []
      self.dict_aruco = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
      self.parameters = aruco.DetectorParameters_create()
      self.detected_aruco = []
      self.matrix_table = None
      #get he folder where to save the homographies
      self.folder = rospy.get_param("calibration_homography")
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
      aruco0 = cv2.imread("/home/altair/odin/src/calibration/arucos/aruco0.png")
      aruco1 = cv2.imread("/home/altair/odin/src/calibration/arucos/aruco1.png")
      aruco2 = cv2.imread("/home/altair/odin/src/calibration/arucos/aruco2.png")
      aruco3 = cv2.imread("/home/altair/odin/src/calibration/arucos/aruco3.png")

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
      pts_table = np.float32([corners[0],corners[1],corners[2],corners[3]])
      self.corners_table_rgb_rev = pts_table

   #project some markers in the corners and detect them
   #the goal is to match the projected markers with the detected ones
   # The detection happens one by one begining by top left, so hide the other ones.
   def calibrate_detected_to_projection(self):
      ok = False
      corners = []
      tmp = []
      print("trying...press c if the detected point are satisfying")
      projected_aruco_img = self.draw_aruco_projector()
      cv2.namedWindow("projection", cv2.WND_PROP_FULLSCREEN)
      cv2.setWindowProperty("projection", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
      #modify second parameter to the X resolution of your screen here
      cv2.moveWindow("projection", 2560,0)
      cv2.imshow("projection", projected_aruco_img)
      cv2.waitKey(50)
      rospy.sleep(5.0)
      s = 0
      while s < 4:
         img, corner, pt = self.detect_single_corner()
         cv2.imshow("detected", img)
         k = cv2.waitKey(0)
         #cv2.imwrite("/home/altair/odin/src/calibration/homography/detected_projected.jpg",img)
         if k == 99:
            tmp.append([corner,pt])
            s += 1
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
      name_zone = "table"#input("name of the zone to calibrate : ")
      self.calibrate_detected_to_projection()
      self.calibrate_projection_to_table()
      print("pts_screen",self.pts_screen)
      print("pts_projected",self.pts_projected)
      print("detected_projected_rev",self.detected_projected_rev)
      print("corners_rev",self.corners_table_rgb_rev)
      #get the matrix transofmr that would be the homopraphies
      m1 = cv2.getPerspectiveTransform(self.pts_screen,self.pts_projected)
      m2 = cv2.getPerspectiveTransform(self.detected_projected_rev,self.pts_projected)
      m3 = cv2.getPerspectiveTransform(self.pts_projected,self.corners_table_rgb_rev)
      projection_matrix = m1.dot(m2).dot(m3)
      projection_matrix_moving = m1.dot(m2)
      name_hom_proj_static = self.home + self.folder + "hom_proj_static.npy"
      name_hom_proj_moving = self.home + self.folder + "hom_proj_moving.npy"
      name_cam_screen = self.home + self.folder + "hom_cam_screen_to_proj.npy"
      name_hom_cam_static = self.home + self.folder + "hom_cam_static.npy"
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
      file_yml = self.home+self.folder+"projection_calibration.yaml"
      with open(file_yml, 'w') as file:
         documents = yaml.dump(data, file)

      print("end script")
      

   def save_matrix(self,name,mat):
      n = "/home/altair/odin/src/calibration/homography/" + name+".npy"
      np.save(n,mat)

   #not used, for debugging
   def test(self):
      pts_screen = np.load("/home/altair/odin/src/calibration/homography/pts_screen.npy")
      pts_projected = np.load("/home/altair/odin/src/calibration/homography/pts_projected.npy")
      detected_projected = np.load("/home/altair/odin/src/calibration/homography/detected_projected.npy")
      corners_table_rgbs = np.load("/home/altair/odin/src/calibration/homography/corners_table_rgb.npy")
      img_detected = cv2.imread("/home/altair/odin/src/calibration/homography/detected_corners_rgb.jpg")
      hom = np.load("/home/altair/odin/src/calibration/homography/hom_proj_moving_table.npy")
      print(corners_table_rgbs)
      rx = 1920/1280
      ry = 1080/720
      tmp = []
      img_projected = self.draw_aruco_projector()
      interface = cv2.imread("/home/altair/odin/src/calibration/homography/interface.jpg")
      projected_coords = [[5,5], [1918, 5], [1918,1076],[5,1076]]
      pts_projected =  np.float32([projected_coords[0],projected_coords[1],projected_coords[2],projected_coords[3]])

      corners_table_rgbs_rev = np.float32([corners_table_rgbs[2],corners_table_rgbs[3],corners_table_rgbs[0],corners_table_rgbs[1]])
      pts_projected_rev = np.float32([pts_projected[2],pts_projected[3],pts_projected[0],pts_projected[1]])
      detected_projected_rev = np.float32([detected_projected[2],detected_projected[3],detected_projected[0],detected_projected[1]])

      m1 = cv2.getPerspectiveTransform(pts_screen,pts_projected)
      m2 = cv2.getPerspectiveTransform(detected_projected_rev,pts_projected)
      m3 = cv2.getPerspectiveTransform(pts_projected,corners_table_rgbs_rev)
      print("pts_screen",pts_screen)
      print("pts_projected",pts_projected)
      print("detected_projected_rev",detected_projected_rev)
      print("corners_rev",corners_table_rgbs_rev)
      m4 = cv2.getPerspectiveTransform(pts_screen,corners_table_rgbs)
      mat = m1.dot(m2).dot(m3)
      out = cv2.warpPerspective(interface,hom,self.screen,flags=cv2.INTER_LINEAR)
      cv2.namedWindow("projection", cv2.WND_PROP_FULLSCREEN)
      cv2.setWindowProperty("projection", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
      cv2.moveWindow("projection", 2560,0)
      cv2.imshow("projection", out)
      cv2.waitKey(50)

   #test code to organize the files as yaml calibration
   def generate_yaml(self):
      hom = np.load("/home/altair/odin/src/calibration/homography/hom_proj_moving.npy")
      name = "table"
      name_hom_proj_static = self.folder + "hom_proj_static.npy"
      name_hom_proj_moving = self.folder + "hom_proj_moving.npy"
      name_cam_screen = self.folder + "hom_cam_screen_to_proj.npy"
      name_hom_cam_static = self.folder + "hom_cam_static.npy"
      np.save(name_hom_proj_static,hom)
      np.save(name_hom_proj_moving,hom)
      np.save(name_cam_screen,hom)
      np.save(name_hom_cam_static,hom)
      n_hps = "hom_proj_static"
      n_hpm = "hom_proj_moving"
      n_hcsp = "hom_cam_screen_to_proj"
      n_hcs = "hom_cam_static"
      #self.save_matrix("hom_proj_static_table",projection_matrix) #for projection on static surface
      #self.save_matrix("hom_proj_moving_table",projection_matrix_moving) #for projection on moving surface
      #self.save_matrix("hom_cam_screen_to_proj",m1) #for projection of interface to projection surface - for detection
      #self.save_matrix("hom_cam_static_table",m3) #transform to static surface - for detection
      name_zone = "table"
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
      file_yml = self.folder+"projection_calibration.yaml"
      with open(file_yml, 'w') as file:
         documents = yaml.dump(data, file)

      print("end script")

      projectors = []
      camera = {}
      with open('/home/altair/odin/src/calibration/homography/projection_calibration.yaml') as file:
         try:
            data = yaml.safe_load(file)
            #print(data)
            for proj in data['proj']:
               tmp = {}
               tmp['id'] = proj['id']
               tmp['zone'] = proj['zone']
               tmp['hom_proj_moving'] = np.load(proj['hom_proj_moving'])
               tmp['hom_proj_static'] = np.load(proj['hom_proj_static'])
               projectors.append(tmp)
            print(tmp)
            for cam in data['cam']:
               tmp = {}
               tmp['id'] = proj['id']
               tmp['zone'] = proj['zone']
               tmp['hom_cam_screen_to_proj'] = np.load(cam['hom_cam_screen_to_proj'])
               tmp['hom_cam_static'] = np.load(cam['hom_cam_static'])
               camera = tmp
               #cameras.append(tmp)
            print(camera)
         except yaml.YAMLError as exception:
            print(exception)



if __name__ == '__main__':
   cal = Calibration()
   #cal.generate_yaml()
   rospy.sleep(2.0)
   #img = cal.draw_aruco_projector()
   cal.system_calibration()
   #while not rospy.is_shutdown():
   #   cv2.namedWindow("test", cv2.WND_PROP_FULLSCREEN)
   #   cv2.setWindowProperty("test", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
   #   cv2.moveWindow("test", 2560,0)
   #   cv2.imshow("test", img)
   #   cv2.waitKey(0)
   #cal.test()
   #cal.detect_aruco_table()
   rospy.spin()