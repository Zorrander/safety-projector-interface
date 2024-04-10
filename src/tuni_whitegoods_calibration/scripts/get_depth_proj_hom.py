#!/usr/bin/env python3
#this code project an aruco grid, detect them on the camera and send them tot he ROS service

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np
from geometry_msgs.msg import Point
import time
import cv2.aruco as aruco
from unity_msgs.msg import poiPCL
import tf2_ros
import tf2_py as tf2
import os
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud,transform_to_kdl
class DepthHom():
	def __init__(self):
		rospy.init_node('calibtest')
		self.m_pos = [] 

		self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
		self.parameters_aruco = aruco.DetectorParameters()
		self.folder = rospy.get_param("calibration_homography")
		self.wait = True
		self.tags = []
		self.size = 100
		self.counter = 0
		self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
		self.tl = tf2_ros.TransformListener(self.tf_buffer)
		self.pointpub = rospy.Publisher("/calibration/pts",poiPCL,queue_size=1)
		self.point_sub = rospy.Subscriber("/calibration/calibrated_points",poiPCL,self.pointsCallback)
		self.rgb_sub = rospy.Subscriber("/rgb/image_raw", Image,self.rgb_callback)
		self.excluded_points = []
		self.points_in_im = {}	
		self.supp = []
		self.ids_output = []
		self.input_points = []
		self.rospoints_input = []
		self.change = False
		self.saveRGBD_flag = True
		self.aruc_id = 0	
		self.depth_raw = None
		self.bridge = CvBridge()
		self.rgb_img = None

	#display the aruco board
	def display_grid(self):
		self.aruc_id = 0	
		self.testim = np.zeros((1080, 1920, 1), dtype="uint8")+255 
		for y in range(10,1010,200):
			for x in range(0,1900,225):
				self.tag = np.zeros((self.size, self.size, 1), dtype="uint8")
				marker_image = aruco.generateImageMarker(self.aruco_dict, self.aruc_id, self.size)
				self.testim[y:y+self.size,x:x+self.size] = marker_image[:, :, np.newaxis]
				self.points_in_im[self.aruc_id] = [x,y]
				self.aruc_id+=1
		#print(aruc_id)
		cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
		#change second parameter to fit your screen resolution
		cv2.moveWindow("window", 2560, 0)
		cv2.setWindowProperty("window", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
		cv2.imshow("window", self.testim)
		cv2.waitKey(100) 

	#only draw the selected markers
	def draw_save_markers(self):
		ending = False
		tmp = np.copy(self.rgb_img)	
		#tmp = cv2.cvtColor(tmp,cv2.COLOR_GRAY2RGB)
		if not self.change:
			#(corners, ids, rejected) = cv2.aruco.detectMarkers(self.rgb_img, arucoDict,parameters=arucoParameters )
			(corners, ids, rejected) = aruco.detectMarkers(self.rgb_img, self.aruco_dict, parameters=self.parameters_aruco )
			#tmp = np.copy(self.rgb_img)	
			if len(corners) > 0:
				ids = ids.flatten()
				aruco.drawDetectedMarkers(tmp, corners, ids=ids, borderColor=(0, 0, 255))
				self.supp = corners
				for i,aruc_id in enumerate(ids):
					if aruc_id not in self.excluded_points:
						self.ids_output.append(aruc_id)
						self.input_points.append(self.points_in_im[aruc_id])
						self.rospoints_input.append(Point(corners[i][0][0][0],corners[i][0][0][1],0.))
		else:
			ids_output = np.array(self.ids_output)
			cv2.aruco.drawDetectedMarkers(tmp, self.supp,ids=ids_output, borderColor=(0, 0, 255))
		#print("excluded points ",self.excluded_points)
		cv2.namedWindow("aruco",cv2.WND_PROP_AUTOSIZE)
		#cv2.setWindowProperty("aruco", cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_NORMAL)
		cv2.imshow("aruco", tmp)
		pressed_key = cv2.waitKey(0)
		if pressed_key == ord('c'):
			pass
		if pressed_key == ord('s'):
			name_f = self.folder + "proj_master_points"
			np.savetxt(name_f,self.input_points)
			pcl_tmp = poiPCL()
			pcl_tmp.pts = self.rospoints_input
			self.pointpub.publish(pcl_tmp)
			#print(ids_output)
			ending = True
		cv2.destroyAllWindows()

		return ending
	#exclude some markers based on user input
	def set_excluded(self,inp):
		tab = inp.split(",")
		for i in tab:
			self.excluded_points.append(int(i))
		ids_out = []
		inp_pts = []
		ros_pts = []
		tmp = []
		for i,aruc_id in enumerate(self.ids_output):
			if aruc_id not in self.excluded_points:
				ids_out.append(aruc_id)
				inp_pts.append(self.points_in_im[aruc_id])
				ros_pts.append(Point(self.supp[i][0][0][0],self.supp[i][0][0][1],0.))
			else:
				tmp.append(i)
				self.change = True
		j = 0
		for i in tmp:
			self.supp = list(self.supp)
			self.supp.pop(i-j)
			j+=1
		#print(self.supp)
		#np.savetxt('ids',ids_output)
		self.ids_output = ids_out
		self.input_points = inp_pts
		self.rospoints_input = ros_pts
		
	#get the transformed points from ROS service
	def pointsCallback(self,data):
		ptss = data.pts
		res = []
		for pts in ptss:
			res.append([pts.x,pts.y])
		print(res)
		name_f = self.folder + "dm_master_points"
		np.savetxt(name_f,res)
		
	#get camera image
	def rgb_callback(self, msg):
		# self.rgb_img = None
		try:
			self.rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)


if __name__ == "__main__":
	dh = DepthHom()
	rospy.sleep(4.0)
	#dh.display_grid()
	#dh.draw_save_markers(True)
	end = False
	#display markers and based on user inputs, exclude some of them
	while not end:
#		val = input("Excluded values: ")
		dh.display_grid()
		rospy.sleep(4.0)
		end = dh.draw_save_markers()
		if not end:
			val = input("Excluded values: ")
			if val != "":
				dh.set_excluded(val)
				rospy.sleep(1.0)

	print("generating homography")
	rospy.sleep(10.0)
	projectors = ['master']
	#generate the homography file
	for proj in projectors:
		name_cp = dh.folder + "proj_{}_points"
		name_cd = dh.folder + "dm_{}_points"
		#coords_proj_f = '/home/altair/odin/src/calibration/homography/proj_{}_points'.format(proj)
		#coords_depth_f = '/home/altair/odin/src/calibration/homography/dm_{}_points'.format(proj)
		coords_proj_f = name_cp.format(proj)
		coords_depth_f = name_cd.format(proj)
		coords_proj = np.loadtxt(coords_proj_f,dtype=np.float32)
		coords_depth = np.loadtxt(coords_depth_f,dtype=np.float32)
		
		M = cv2.findHomography(coords_depth,coords_proj)[0]
		name_save = dh.folder + "depth_proj_{}"
		#np.save('/home/altair/odin/src/calibration/homography/depth_proj_{}'.format(proj),M)
		np.save(name_save.format(proj),M)
	print("generated homography")
	rospy.spin()
