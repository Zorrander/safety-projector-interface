#!/usr/bin/env python3
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
		self.home = self.home = os.environ.get("HOME")
		self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
		self.parameters_aruco = aruco.DetectorParameters_create()
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

	def display_grid(self):
		self.aruc_id = 0	
		self.testim = np.zeros((1080, 1920, 1), dtype="uint8")+255 
		for y in range(10,1010,200):
			for x in range(0,1900,225):
				self.tag = np.zeros((self.size, self.size, 1), dtype="uint8")
				cv2.aruco.drawMarker(self.aruco_dict, self.aruc_id, self.size, self.tag, 1)
				self.testim[y:y+self.size,x:x+self.size] = self.tag
				self.points_in_im[self.aruc_id] = [x,y]
				self.aruc_id+=1
		#print(aruc_id)
		cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
		cv2.moveWindow("window", 2560, 0)
		cv2.setWindowProperty("window", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
		cv2.imshow("window", self.testim)
		cv2.waitKey(100) 

	def draw_save_markers(self):
		ending = False
		tmp = np.copy(self.rgb_img)	
		#tmp = cv2.cvtColor(tmp,cv2.COLOR_GRAY2RGB)
		if not self.change:
			arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
			arucoParameters = cv2.aruco.DetectorParameters_create()
			#(corners, ids, rejected) = cv2.aruco.detectMarkers(self.rgb_img, arucoDict,parameters=arucoParameters )
			(corners, ids, rejected) = cv2.aruco.detectMarkers(self.rgb_img, arucoDict,parameters=arucoParameters )
			#tmp = np.copy(self.rgb_img)	
			if len(corners) > 0:
				ids = ids.flatten()
				cv2.aruco.drawDetectedMarkers(tmp, corners,ids=ids, borderColor=(0, 0, 255))
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
			name_f = self.home + self.folder + "proj_master_points"
			np.savetxt(name_f,self.input_points)
			pcl_tmp = poiPCL()
			pcl_tmp.pts = self.rospoints_input
			self.pointpub.publish(pcl_tmp)
			#print(ids_output)
			ending = True
		cv2.destroyAllWindows()

		return ending
	
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
			self.supp.pop(i-j)
			j+=1
		#print(self.supp)
		#np.savetxt('ids',ids_output)
		self.ids_output = ids_out
		self.input_points = inp_pts
		self.rospoints_input = ros_pts
		

	def pointsCallback(self,data):
		ptss = data.pts
		res = []
		for pts in ptss:
			res.append([pts.x,pts.y])
		print(res)
		name_f = self.home + self.folder + "dm_master_points"
		np.savetxt(name_f,res)

	def rgb_callback(self, msg):
		self.rgb_img = None
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
	rospy.sleep(5.0)
	projectors = ['master']
	
	for proj in projectors:
		name_cp = dh.home + dh.folder + "proj_{}_points"
		name_cd = dh.home + dh.folder + "dm_{}_points"
		#coords_proj_f = '/home/altair/odin/src/calibration/homography/proj_{}_points'.format(proj)
		#coords_depth_f = '/home/altair/odin/src/calibration/homography/dm_{}_points'.format(proj)
		coords_proj_f = name_cp.format(proj)
		coords_depth_f = name_cd.format(proj)
		coords_proj = np.loadtxt(coords_proj_f,dtype=np.float32)
		coords_depth = np.loadtxt(coords_depth_f,dtype=np.float32)
		
		M = cv2.findHomography(coords_depth,coords_proj)[0]
		name_save = dh.home + dh.folder + "depth_proj_{}"
		#np.save('/home/altair/odin/src/calibration/homography/depth_proj_{}'.format(proj),M)
		np.save(name_save.format(proj),M)
	print("generated homography")
	rospy.spin()
