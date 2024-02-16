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
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud,transform_to_kdl
class DepthHom():
	def __init__(self):
		rospy.init_node('calibtest')
		self.m_pos = [] 
		self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
		self.parameters_aruco = aruco.DetectorParameters_create()
		self.wait = True
		self.tags = []
		self.size = 100
		self.counter = 0
		self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
		self.tl = tf2_ros.TransformListener(self.tf_buffer)
		self.pointpub = rospy.Publisher("/calibration/pts",poiPCL,queue_size=1)
		self.point_sub = rospy.Subscriber("/calibration/calibrated_points",poiPCL,self.pointsCallback)
		self.excluded_points = []
		self.points_in_im = {}	
		self.supp = []
		self.ids_output = []
		self.input_points = []
		self.rospoints_input = []
		self.change = False
		#for i in range(4):
		    #tag = np.zeros((size, size, 1), dtype="uint8")
		    #cv2.aruco.drawMarker(self.aruco_dict, i, size, tag, 1)
		    #tags.append(tag)
		self.saveRGBD_flag = True
		#points = [[300,300],[int(1080/2-size/2+0.5),int(1920/2-size/2+0.5)],[700,1300],[200,1300]]
		#points = [[10,100],[10,size+110],[10,2*size+110],[10,3*size+110],[10,1920-size-10],[1080-10-size-10,1920-size-10],[1080-size-10,10]]
		#points = [[10,2*size+110],[10,1920-size-10],[1080-10-size-10,1920-size-10],[1080-size-10,10+2*size]]
		self.aruc_id = 0	
		
		#self.testim = np.zeros((1080, 1920, 1), dtype="uint8")+255 	
		#self.testim[...] = 0
		#for i in range(len(points)):
		#    tag = np.zeros((size, size, 1), dtype="uint8")
		#    cv2.aruco.drawMarker(self.aruco_dict, i, size, tag, 1)
		#    tags.append(tag)
		#%%
		#self.testim = np.zeros((1080, 1920, 1), dtype="uint8")+255 
		#for i,point in enumerate(points):
		#    self.testim[point[0]:point[0]+size,point[1]:point[1]+size] = tags[i]
		
		#cv2.waitKey(0) 
		self.depth_raw = None
		self.bridge = CvBridge()
		self.rgb_sub = rospy.Subscriber("/master/rgb/image_raw", Image,self.img_cb)

	def display_grid(self):
		self.aruc_id = 0	
		self.testim = np.zeros((1080, 1920, 1), dtype="uint8")+255 
		for y in range(100,1100,200):
			for x in range(0,1900,225):
				self.tag = np.zeros((self.size, self.size, 1), dtype="uint8")
				cv2.aruco.drawMarker(self.aruco_dict, self.aruc_id, self.size, self.tag, 1)
				self.testim[y:y+self.size,x:x+self.size] = self.tag
				self.points_in_im[self.aruc_id] = [x,y]
				self.aruc_id+=1
		#print(aruc_id)
		cv2.namedWindow("window", cv2.WND_PROP_FULLSCREEN)
		#cv2.moveWindow("window", 2560, 0)
		cv2.setWindowProperty("window", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
		cv2.imshow("window", self.testim)
		cv2.waitKey(100) 

	def draw_save_markers(self):
		#self.rgb_img = None
		#try:
			#self.rgb_img = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
		#except CvBridgeError as e:
		#	print(e)
		ending = False
		tmp = np.copy(self.testim)	
		tmp = cv2.cvtColor(tmp,cv2.COLOR_GRAY2RGB)
		if not self.change:
			arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
			print(cv2.aruco.DICT_4X4_50)
			arucoParameters = cv2.aruco.DetectorParameters_create()
			#(corners, ids, rejected) = cv2.aruco.detectMarkers(self.rgb_img, arucoDict,parameters=arucoParameters )
			(corners, ids, rejected) = cv2.aruco.detectMarkers(self.testim, arucoDict,parameters=arucoParameters )
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
			np.savetxt('/home/altair/odin/src/calibration/homography/proj_points',self.input_points)
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
				print("excluded : ",aruc_id)
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
		np.savetxt('dm_points',res)

	def img_cb(self, rgb_data):
		self.rgb_img = None
		try:
			self.rgb_img = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
		except CvBridgeError as e:
			print(e)
		arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
		arucoParameters = cv2.aruco.DetectorParameters_create()
		(corners, ids, rejected) = cv2.aruco.detectMarkers(self.rgb_img, arucoDict,parameters=arucoParameters )
		tmp = np.copy(self.rgb_img)	
		if len(corners) > 0:
			ids = ids.flatten()
			cv2.aruco.drawDetectedMarkers(tmp, corners)
		cv2.imshow("ArUCo Tag", tmp)
		pressed_key = cv2.waitKey(0)
		if pressed_key == ord('s'):
			ids_output = []
			input_points = []
			rospoints_input = []
			for i,aruc_id in enumerate(ids):
				ids_output.append(aruc_id)
				input_points.append(self.points_in_im[aruc_id])
				rospoints_input.append(Point(corners[i][0][0][0],corners[i][0][0][1],0.))
			#np.savetxt('ids',ids_output)
			np.savetxt('proj_points',input_points)
			pcl_tmp = poiPCL()
			pcl_tmp.pts = rospoints_input
			self.pointpub.publish(pcl_tmp)

		return


if __name__ == "__main__":
	dh = DepthHom()
	rospy.sleep(2.0)
	#dh.display_grid()
	#dh.draw_save_markers(True)
	end = False
	while not end:
#		val = input("Excluded values: ")
		dh.display_grid()
		rospy.sleep(2.0)
		end = dh.draw_save_markers()
		if not end:
			val = input("Excluded values: ")
			dh.set_excluded(val)
	rospy.spin()
