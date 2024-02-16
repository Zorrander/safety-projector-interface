#!/usr/bin/env python3
from multiprocessing.resource_sharer import stop
import cv2
import sys
import time
import os
from gpg import Data
import numpy as np
import rospy
import yaml
import json
import rospkg
import pprofile
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from sensor_msgs.msg import Image
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../../robot/scripts/')
from ur5_kinematics import ur5

from sensor_msgs.msg import JointState
from unity_msgs.msg import MarkerDataArray
from unity_msgs.msg import ManipulatedObject
from unity_msgs.msg import HomographyMtx
from unity_msgs.msg import DataProj
from unity_msgs.msg import Projection
from unity_msgs.msg import ListProjection
from unity_msgs.msg import ListDataProj
from unity_msgs.msg import ArucoArray
from unity_msgs.msg import ArucoMarker
from unity_msgs.msg import InterfacePOI
from unity_msgs.msg import ElementUI
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from geometry_msgs.msg import Point

# class for drawing simple images with text at given location 
class Pattern():
    def __init__(self, id, texture, location, size):
        self._texture = texture
        self._location = location
        self._id = id
        self._H, self._W = self._texture.shape[:2]
        self._count_time = None
        self._center = (int(location[1]+size[0]/2),int(location[0]+size[1]/2))

    def get_center(self):
        return self._center

    def draw_pattern(self, img, active=True):
        if active:
            temp = self._texture.copy()
        else:
            temp = self._texture.copy()
            temp[temp > 100] = 100
        img[self._location[0]:self._location[0] + self._H, self._location[1]:self._location[1] + self._W] = temp
        return img

    def draw_pattern_with_text(self, img, text='Robot action:'):
        img[self._location[0]:self._location[0] + self._H, self._location[1]:self._location[1] + self._W] = self._texture.copy()
        cv2.putText(img, text, (self._location[1] + 80, (self._location[0] - 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), lineType=cv2.LINE_AA, thickness=2)
        return img

    def draw_pattern_with_counter(self, img):
        if self._count_time is None:
            self._count_time = time.time() + 8.0
        time_left = self._count_time - time.time()
        if time_left < 0.0:
            time_left = 0.00
        img[self._location[0]:self._location[0] + self._H, self._location[1]:self._location[1] + self._W] = self._texture.copy()
        cv2.putText(img, "Grab the rocker shaft, going", (self._location[1] - 40, (self._location[0] - 55)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), lineType=cv2.LINE_AA, thickness=2)
        cv2.putText(img, "into force mode in " + "{:0.2f}".format(time_left) +
                    " sec.",  (self._location[1] - 40, (self._location[0] - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), lineType=cv2.LINE_AA, thickness=2)
        return img

#class for interface image
class Interface():
    def __init__(self):
        self._components = {}

    def get_components_center(self):
        return self._components

    def read_component(self, id, texture_path, location, size):
        texture = cv2.imread(texture_path)
        self._components[id] = Pattern(id, texture, location, size)

    def read_config(self, path):
        with open(path + 'projector_buttons.yaml', 'r') as f:
            data = yaml.safe_load(f)
        keys = list(data.keys())
        for i in range(len(keys)):
            self.read_component(keys[i], path + data[keys[i]]['img_path'], data[keys[i]]['loc'], data[keys[i]]['size'])

    def draw(self, screen, robot_state, button_state, system_state):
        if system_state == 'force_mode':
            screen = self._components['GO'].draw_pattern(screen, True)
            screen = self._components['STOP'].draw_pattern(screen, False)
            screen = self._components['CONFIRM'].draw_pattern(screen, False)
            screen = self._components['DEAD_MAN'].draw_pattern(screen, True)
            screen = self._components['START_ROBOT'].draw_pattern(screen, False)
            screen = self._components['GRASP_ARM'].draw_pattern_with_counter(screen)
        if robot_state == 'moving':
            screen = self._components['GO'].draw_pattern(screen, False)
            screen = self._components['STOP'].draw_pattern(screen, True)
            screen = self._components['CONFIRM'].draw_pattern(screen, False)
            screen = self._components['DEAD_MAN'].draw_pattern(screen, False)
            screen = self._components['ROBOT_ACTIVE'].draw_pattern(screen, True)
        elif robot_state == 'stationary':
            screen = self._components['GO'].draw_pattern(screen, True)
            screen = self._components['STOP'].draw_pattern(screen, False)
            screen = self._components['CONFIRM'].draw_pattern(screen, False)
            screen = self._components['DEAD_MAN'].draw_pattern(screen, True)
            if button_state == 'dead_man_not_pressed':
                screen = self._components['DM_NOT_PRESSED'].draw_pattern(screen, True)
            else:
                screen = self._components['START_ROBOT'].draw_pattern(screen, True)
        else:
            print("Unkown robot state")

        if system_state == 'confirm_motor_frame':
            screen = self._components['MOTOR_FRAME'].draw_pattern_with_text(screen)

        if system_state == 'confirm_rocker_shaft':
            screen = self._components['ROCKER_SHAFT'].draw_pattern_with_text(screen)

        return screen

#class for visualizing with projector
class Projector():
    def __init__(self, interface_configs_path, common_configs, homogprahy_path):
        self.links = ['base_link','link_4','link_5','flange','tool0']
        linksubs = []
        for link in self.links:
            linksubs.append(message_filters.Subscriber('coords/'+link, TransformStamped))#topics for coords of different joints
        self.joint_ts = message_filters.ApproximateTimeSynchronizer(linksubs, 1, 2)
        self.joint_ts.registerCallback(self.cb_joints)
        self.projectors = []
        self.mainCamera = {}
        #name = "/home/altair/catkin_abb/src/projector/"
        name = ""
        with open('/home/odin1/odin/src/projector/projector_devices_master.json') as f: #data for projectors and the kinect
        #with open('/home/altair/odin/src/projector/projector_devices_master2.json') as f:
            prj_json = json.load(f)
            for prj_data in prj_json['projectors']:
                tmp = {}
                tmp['id'] = prj_data['id']
                tmp['directHom'] = np.load(name+prj_data['homDirect']) 
                tmp['vertHom'] = np.load(name+prj_data['homFromVert'])#homography for vertical view of RGB image on ground level
                tmp['vertHomTable'] = np.load(name+prj_data['homTableVert'])#homography for vertical view of RGB image on table level
                tmp['homDepthProj'] = np.load(name+prj_data['homDepthProj'])
                tmp['shiftX'] = prj_data['shiftX']#shifting opencv window to the projector
                self.projectors.append(tmp)
                #break
            for cam_data in prj_json['cameras']:
                if cam_data['mainCam'] == 1:
                    self.mainCamera['vertHomGround'] = np.load(name+cam_data['vertHomGround'])#homography from normal image to vertical view on ground level
                    self.mainCamera['camRobHom'] = np.load(name+cam_data['camRobHom'])#homography from robot coords to image
                    self.mainCamera['vertHomTable'] = np.load(name+cam_data['vertTable'])#homography from normal image to vertical view on table level
        self.is_init = False
        self.depthIm = None
        self.rgb_img = None
        self.depth2rgb = None
        self.aruco_prev_center_point = np.array([-1000.,-1000.])
        self.table_space_transform = None
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters_aruco = aruco.DetectorParameters_create()
        #self.rgb_sub = message_filters.Subscriber("/sub/rgb/image_rect_color", Image)
        #self.depth_sub = message_filters.Subscriber("/sub/depth/image_rect", Image)
        self.rgb_sub = message_filters.Subscriber("/master/rgb/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/master/depth/image_raw", Image)
        self.depth2RGB_sub = message_filters.Subscriber("/master/depth_to_rgb/image_raw", Image)
        self.UI_transform = None
        im_subs = [self.rgb_sub,self.depth_sub,self.depth2RGB_sub]
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.depth2RGB_sub], 1, 2)
        self.ts.registerCallback(self.img_cb_once,im_subs)
        self._table_homography_pub = rospy.Publisher("/unity/table_homography", HomographyMtx, queue_size=1)#homography for the table to do visualization
        #self._joint_sub = rospy.Subscriber('joint_states', JointState, self.cb_joint_state)
        self._marker_sub = rospy.Subscriber("/unity/interaction_markers", MarkerDataArray, self.cb_markers_state)
        self._system_state_sub = rospy.Subscriber("/unity/system_mode", String, self.cb_system_state)#getting state of the robot program
        self._object_manipulation_sub = rospy.Subscriber("/unity/manipulated_object", ManipulatedObject,
                                                   self.cb_object_manipulation)#not used at the moment
        self._screen_size = (1080, 1920)
        #self._vert_view_size = (1280,720) #change to 1024 1024
        self._vert_view_size = (1024,1024)
        self._current_joint_values = None
        self.stop = 0
        self.go = 0
        self.confirm = 0
        self.tg = 0
        self.dm = 0
        self.matrix = 0
        #self._robot_kin = ur5()
        self._interface = Interface()
        self._robot_state = 'stationary'
        self.c_points = None
        self._cfg = None
        self._button_info = None
        self._system_state = None
        self._robot_carrying_object = False
        self._H = np.loadtxt(homogprahy_path)
        self.bridge_interface = CvBridge()
        self.projection_list_pub = rospy.Publisher("/list_projection", ListProjection, queue_size=1)
        self.dp_list_pub = rospy.Publisher("/list_dp", ListDataProj, queue_size=1)
        self.pub_poi = rospy.Publisher("/interface_poi/test", InterfacePOI, queue_size=1)
        self.marker_array_sub = rospy.Subscriber("/aruco_markers_warped", ArucoArray, self.callbackArrayMarkers)
        self.sub_safety_line = rospy.Subscriber("/safety_line/line_proj", Image, self.callback_safety_line)
        self.arr_markers = ArucoArray()
        self.marker = ArucoMarker()
        self.prev_tlp = [0,0]
        self.prev_blp = [0,0]
        self.detect_change = True
        self.first = True
        self.sl_dm = None
        self.s_marker = 0
        self.received = False

        self.init(interface_configs_path, common_configs)
        rospy.loginfo("Projector interface initialized!")

    def callbackArrayMarkers(self,msg):
        self.arr_markers = msg.array_markers
        self.s_marker = len(self.arr_markers)

    def cb_joints(self,*data):#get coordinates of the joints
        #print("hey !")
	#print(len(data))
        num_of_joints = len(data)
        self.c_points = np.ones((num_of_joints,4))
        for i in range(num_of_joints):
            xyz_data = data[i]
            self.c_points[i] =[xyz_data.transform.translation.x,xyz_data.transform.translation.y,xyz_data.transform.translation.z,1]
        self.c_points = self.c_points.T
        #print(self.c_points)

    def img_cb_once(self, rgb_data, depth_data,depth2RGB_data,subscribers):
        self.depthIm = CvBridge().imgmsg_to_cv2(depth2RGB_data, "passthrough")
        self.rgb_img = CvBridge().imgmsg_to_cv2(rgb_data, "bgr8")
        self.depth2rgb = CvBridge().imgmsg_to_cv2(depth2RGB_data, "passthrough")
	#[sub.sub.unregister() for sub in subscribers]

    def callback_safety_line(self,msg):
        self.sl_dm = CvBridge().imgmsg_to_cv2(msg, "passthrough")
        #self.sl_dm = cv2.cvtColor(tmp,cv2.COLOR_GRAY2RGB)
        self.received = True

    def init(self, interface_configs_path, common_configs):
        with open(common_configs, 'r') as ymlfile:
            self._cfg = yaml.load(ymlfile, Loader=yaml.SafeLoader)
        self._interface.read_config(interface_configs_path)

    def cb_object_manipulation(self, msg):
        if msg.id == 0:
            self._robot_carrying_object = False
        else:
            self._robot_carrying_object = True

    def cb_system_state(self, msg):
        self._system_state = msg.data

    def cb_markers_state(self, msg):#data for interaction elements
        go_active = False
        confirm_active = False
        dead_man_active = False
        for marker in msg.markers:
            if marker.id == 'go':
                if marker.data > self._cfg['interaction_button_thres']:
                    go_active = True

            if marker.id == 'confirm':
                if marker.data > self._cfg['interaction_button_thres']:
                    confirm_active = True

            if marker.id == 'dead_man':
                if marker.data > self._cfg['interaction_button_thres']:
                    dead_man_active = True

        if dead_man_active == False and go_active or confirm_active:
            self._button_info = 'dead_man_not_pressed'
        else:
            self._button_info = None

    def cb_joint_state(self, msg):
        self._current_joint_values = np.array(msg.position)
        joint_speeds = np.array(msg.velocity)

        if np.sum(abs(joint_speeds)) > 1e-1:
            self._robot_state = 'moving'
        else:
            self._robot_state = 'stationary'

    def points_in_circum(self, r, ox, oy, n=50):
        return [(np.cos(2 * np.pi / n * x) * r + ox, np.sin(2 * np.pi / n * x) * r + oy) for x in range(0, n + 1)]

    def generate_hull(self, circle_radius, offset,contour_thickness):
        if offset == 0:
            offset = [0,0]
        cs = self.c_points #3D coord of joints in base_frame
        #replace from here by calling service to transform 3D points robot_base to depth_map coords
        H = self.mainCamera['camRobHom']
        color_mask = np.zeros((self._vert_view_size[1],self._vert_view_size[0], 3), np.uint8)
        mask = np.zeros((self._vert_view_size[1],self._vert_view_size[0]), np.uint8) #kinect camera resolution, TODO read from parameters
        point_joints = np.dot(H,cs)
        point_joints = point_joints / point_joints[2, :]
        point_joints = np.dot(self.mainCamera['vertHomGround'],point_joints)
        point_joints = point_joints / point_joints[2, :]
        point_joints = point_joints[:2,:].astype(int)
        point_joints = point_joints.T
        #to here
        for c in point_joints:#make circular areas around joints
            cv2.circle(mask, (c[0]+offset[0],c[1]+offset[1]), circle_radius, (255,), -1)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)#generate contours
        merged_contour = []
        if len(contours) != 0:
            for c1 in contours:
                for c2 in c1:
                    merged_contour.append(c2)
        hull = cv2.convexHull(np.array(merged_contour), False)
        cv2.drawContours(color_mask, [hull], 0, (0, 0, 255), contour_thickness)
        return color_mask

    def work_object_cpoints(self, ee_frame):#not used
        obj = np.eye(4)
        obj[1, 3] = self._cfg['work_object_width'] / 2  # 0.12
        obj1_base_frame = np.dot(ee_frame, obj)[0:2, 3]  # take only x and y components
        obj[1, 3] = (-1.0) * self._cfg['work_object_width'] / 2
        obj2_base_frame = np.dot(ee_frame, obj)[0:2, 3]
        return np.array([obj1_base_frame, obj2_base_frame])
    def scaleRectangle(self, points,scaler, center = None):#not used
        points = np.array(points)
        if center is None:
            center = points[0]+(points[2]-points[0])/2
        tmp = points-center
        points_scaled = tmp*scaler+center
        return points_scaled.astype(np.int32)

    def publishUITransform(self):
        d = DataProj()
        d.transform = self.UI_transform.flatten()
        self.transform_pub.publish(d)

    def find_ui_transform(self):#find homography for the table
        #print(self.rgb_img)
        #gray = cv2.cvtColor(self.rgb_img,cv2.COLOR_BGR2GRAY)
        M = self.mainCamera['vertHomTable']#transform to vertical view
        #proj_space=cv2.warpPerspective(self.depthIm,M,(self._vert_view_size[0],self._vert_view_size[1]))
        #imagefortest=cv2.warpPerspective(self.rgb_img,M,(self._vert_view_size[0],self._vert_view_size[1]))
        #replaced imagefortest by RGB camera
        detect_marker = False
        #if len(self.arr_markers[0].tl_corner) > 0:
        if self.s_marker > 0:
            detect_marker = True
            #Alternative aruco values from the c++ aruco detector
            tl_point = np.array([self.arr_markers[0].tl_corner[0],self.arr_markers[0].tl_corner[1]])
            tr_point = np.array([self.arr_markers[0].tr_corner[0],self.arr_markers[0].tr_corner[1]])
            br_point = np.array([self.arr_markers[0].br_corner[0],self.arr_markers[0].br_corner[1]])
            bl_point = np.array([self.arr_markers[0].bl_corner[0],self.arr_markers[0].bl_corner[1]])
            diff_tl_x = abs(self.prev_tlp[0] - tl_point[0])
            diff_tl_y = abs(self.prev_tlp[1] - tl_point[1])
            diff_br_x = abs(self.prev_blp[0] - br_point[0])
            diff_br_y = abs(self.prev_blp[1] - br_point[1])
            v_l = [abs(tl_point[0] - bl_point[0]),abs(tl_point[1] - bl_point[1])]
            v_r = [abs(tr_point[0] - br_point[0]),abs(tr_point[1] - br_point[1])]
            c_x = v_l[0] / v_r[0]
            c_y = v_l[1] / v_r[1]
            t = abs(c_x - c_y)

        #detect changes
        if detect_marker == True and t > 0.021 and (diff_tl_x > 1 or diff_tl_y > 1 or diff_br_x > 1 or diff_br_y > 1) :
            self.detect_change = True
        else:
            self.detect_change = False

        if (self.detect_change or self.first) and self.s_marker > 0:
            self.first = False
            aruco_long_vec = np.array([self.arr_markers[0].width_marker[0],self.arr_markers[0].width_marker[1]])
            aruco_short_vec = np.array([self.arr_markers[0].height_marker[0],self.arr_markers[0].height_marker[1]])
            ##big table
            scale_long = 6.4#compute size of the marker relative to the size of the table to find table corners
            scale_short = 4.2
            tr_point_t = tl_point+aruco_long_vec*scale_long
            bl_point_t = tl_point+aruco_short_vec*scale_short
            br_point_t = tl_point+aruco_short_vec*scale_short+aruco_long_vec*scale_long


            #img_contours = imagefortest #this used for tests, can be deleted later
            pts1 = np.float32([[0,0],[0,self._screen_size[0]],[self._screen_size[1],0]])#corners of the screen
            pts2 = np.float32([tl_point,bl_point_t,tr_point_t])#corners on the image in Kinect
            pts3 = np.float32([tl_point,tr_point_t,br_point_t,bl_point_t])
            #pts4 = np.float32([[0,0],[self._screen_size[1],0],[0,self._screen_size[0]],[self._screen_size[1],self._screen_size[0]]])
            self.UI_transform = cv2.getAffineTransform(pts1,pts2)
            #print("UI : ",self.UI_transform)
            tmp_transform = np.zeros((3,3))
            tmp_transform[:2,...] = self.UI_transform
            tmp_transform[2,2] = 1
            self.UI_transform = tmp_transform

            #do it with getPerspective
            #print("GET Perspective ")
            #pts3 = np.float32([tl_point,tr_point_t,bl_point_t,br_point_t])
            pts4 = np.float32([[0,0],[self._screen_size[1],0],[self._screen_size[1],self._screen_size[0]],[0,self._screen_size[0]]])#screen's corners
            self.matrix = cv2.getPerspectiveTransform(pts4,pts3)
            #self.UI_transform = cv2.getPerspectiveTransform(pts4,pts3)

            #updating values for detection
            self.prev_tlp[0] = tl_point[0]
            self.prev_tlp[1] = tl_point[1]
            self.prev_blp[0] = br_point[0]
            self.prev_blp[1] = br_point[1]

        return

    def get_button_position(self,b):
        px = (self.matrix[0][0]*b[0] + self.matrix[0][1]*b[1] + self.matrix[0][2]) / ((self.matrix[2][0]*b[0] + self.matrix[2][1]*b[1] + self.matrix[2][2]))
        py = (self.matrix[1][0]*b[0] + self.matrix[1][1]*b[1] + self.matrix[1][2]) / ((self.matrix[2][0]*b[0] + self.matrix[2][1]*b[1] + self.matrix[2][2]))
        button = (int(px), int(py))
        
        return button

    def fillLayoutMessage(self,l_poi):
        poi = self._interface.get_components_center()
        M = self.mainCamera['vertHomTable']
        inv = np.linalg.inv(M)
        l_pts = []
        
        #first loop to generate button position to RGB
        for i in poi.keys():
            if i == "GO" or i == "STOP" or i == "DEAD_MAN" or i == "CONFIRM":
                t = poi[i].get_center()
                #print(t)
                tmp_pt = self.get_button_position(t)
                tmp_i = [tmp_pt[0],tmp_pt[1]]
                l_pts.append(tmp_i)
        s = np.array(l_pts, dtype='float32')
        st = np.array([s])
        p = cv2.perspectiveTransform(st,inv)
        #second loop to fill info and send them
        k = 0
        for j in poi.keys():
            if j == "GO" or j == "STOP" or j == "DEAD_MAN" or j == "CONFIRM":
                tmp_elem = ElementUI()
                tmp_elem.id = j
                tmp_elem.elem.x = int(p[0,k,0])
                tmp_elem.elem.y = int(p[0,k,1])
                k += 1
                l_poi.poi.append(tmp_elem)
        self.pub_poi.publish(l_poi)
        
        
    def run(self):
        last_check = 0.0
        robot_state = None
        dst = [0,0]
        save = True
        count = 0
        while not rospy.is_shutdown():
            interface_img = np.zeros((self._screen_size[0], self._screen_size[1], 3), np.uint8)
            tmp = np.zeros((self._screen_size[0], self._screen_size[1], 3), np.uint8)
            #image_warped=cv2.warpPerspective(self.rgb_img,M,(self._vert_view_size[0],self._vert_view_size[1]))

            # Generate interface components
            if time.time() - last_check > 0.3:
                robot_state = self._robot_state #robot state does not change now
                last_check = time.time()
            interface_tmp = self._interface.draw(tmp, robot_state, self._button_info, self._system_state)#generate interface

            self.find_ui_transform()#find transformation to the table
            #self._table_homography_pub.publish(self.table_space_transform.flatten())#pubshish inverse transform for detecting button interactions
            #generate and send message of POI position in RGB space
            l_poi = InterfacePOI()
            if self.s_marker > 0:
                self.fillLayoutMessage(l_poi)

            #print("Transform ")
            #print(self.UI_transform)
            M = self.mainCamera['vertHomTable']
            self._table_homography_pub.publish(M.flatten())
            # Generate hull
            #joint_values = self._current_joint_values.copy()
            if self.c_points is not None:
                safety_line = self.generate_hull(90, [-60,-60],5)#generate safety line if topics exist
            else:
                safety_line = np.zeros((720,1280, 3), np.uint8)
            #interface_img += safety_line #sum them up
            #pr_list = ListProjection()
            dp_list = ListDataProj()
            #test to display buttons
            #cv2.namedWindow("Buttons_d", cv2.WINDOW_NORMAL)
            """if save == True:
                cv2.imwrite("/home/odin1/odin/src/projector/cpp_sl.jpg",self.sl_dm)
                cv2.imwrite("/home/odin1/odin/src/projector/python_sl.jpg",safety_line)
                save = False"""
            
            #Publish matrix transform and image for each projector
            for proj in self.projectors:#iterate over projectors and display content for each
                #if proj['id'] == 0:
                if self.s_marker > 0:
                    dp_interface = DataProj()
                    dp_interface.id = proj['id']
                    tmp = np.matmul(proj['vertHomTable'], self.UI_transform)
                    dp_interface.transform = tmp.flatten()
                    dp_interface.img = self.bridge_interface.cv2_to_imgmsg(interface_tmp, "bgr8")
                    dp_list.list_proj.append(dp_interface)

                    """im_tmp = cv2.warpPerspective(interface_tmp, np.matmul(proj['vertHomTable'], self.UI_transform),(1920,1080))#UI on table level
                    p = (145,878) #145 878
                    px = (self.matrix[0][0]*p[0] + self.matrix[0][1]*p[1] + self.matrix[0][2]) / ((self.matrix[2][0]*p[0] + self.matrix[2][1]*p[1] + self.matrix[2][2]))
                    py = (self.matrix[1][0]*p[0] + self.matrix[1][1]*p[1] + self.matrix[1][2]) / ((self.matrix[2][0]*p[0] + self.matrix[2][1]*p[1] + self.matrix[2][2]))
                    self.stop = (int(px), int(py))
                    test = cv2.circle(self.rgb_img, self.stop, radius=5, color=(0, 0, 255), thickness=-1)"""
                    
                    
                    
                    #safety_tmp = cv2.warpPerspective(safety_line,proj['vertHom'],(1920,1080))#border on ground level
                    #im_tmp = cv2.add(im_tmp,safety_tmp)
                    #cv2.imshow("interface", test)
                    #v2.waitKey(1)
                if self.received:
                    dp_safety = DataProj()
                    dp_safety.id = proj['id']
                    dp_safety.transform = proj['homDepthProj'].flatten()
                    #print(dp_safety.transform)
                    dp_safety.img = self.bridge_interface.cv2_to_imgmsg(self.sl_dm, "bgr8")#self.sl_dm
                    dp_list.list_proj.append(dp_safety)
                
                
            save = False

                #tmp_img = cv2.warpPerspective(interface_tmp, np.matmul(proj['vertHomTable'], self.UI_transform),(1920,1080))#UI on table level
                #safety_tmp = cv2.warpPerspective(safety_line,proj['vertHom'],(1920,1080))#border on ground level
                #complete_img = cv2.add(tmp_img,safety_tmp)
                    

            self.dp_list_pub.publish(dp_list)

            """for proj in self.projectors:#iterate over projectors and display content for each
                tmp = np.matmul(proj['vertHomTable'], self.UI_transform)
                windowname = "window"+str(proj['id'])
                cv2.namedWindow(windowname, cv2.WND_PROP_FULLSCREEN)
                cv2.moveWindow(windowname, proj['shiftX'], 0)#shift opencv window to projector
                cv2.setWindowProperty(windowname, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                im_tmp = cv2.warpPerspective(interface_tmp, np.matmul(proj['vertHomTable'], self.UI_transform),(1920,1080))#UI on table level
                #stop_button = (878,145)
                #t = np.matmul(stop_button,self.UI_transform)
                #self.stop = (int(t[0]),int(t[1]))
                p = (145,978) #145 878
                px = (self.matrix[0][0]*p[0] + self.matrix[0][1]*p[1] + self.matrix[0][2]) / ((self.matrix[2][0]*p[0] + self.matrix[2][1]*p[1] + self.matrix[2][2]))
                py = (self.matrix[1][0]*p[0] + self.matrix[1][1]*p[1] + self.matrix[1][2]) / ((self.matrix[2][0]*p[0] + self.matrix[2][1]*p[1] + self.matrix[2][2]))
                self.stop = (int(px), int(py))
                test = cv2.circle(self.rgb_img, self.stop, radius=5, color=(0, 0, 255), thickness=-1)
                im_tmp = cv2.warpPerspective(cv_img, np.matmul(proj['vertHomTable'], self.UI_transform),(1920,1080))
                
                #safety_tmp = cv2.warpPerspective(safety_line,proj['vertHom'],(1920,1080))#border on ground level
                #im_tmp = cv2.add(im_tmp,safety_tmp)
                cv2.imshow(windowname, im_tmp)
                cv2.waitKey(1)"""

def main():
    rospy.init_node('robot_safety_area_node')
    rospack = rospkg.RosPack()
    config_prefix = rospack.get_path('unity_msgs') + '/configs/'
    interface_config_path = config_prefix + 'mobile_demo/'
    common_config_path = config_prefix + 'config.yaml'
    homography_file = config_prefix + 'robot_projector_homography.txt'
    proj = Projector(interface_config_path, common_config_path, homography_file)
    rospy.sleep(1.0)
    proj.run()


if __name__ == "__main__":
	main()
	#print(os.environ)
	#prof = pprofile.StatisticalProfile()
	#with prof():
	#	main()
	#with open('/home/robolab/catkin_ws/profiling_results.bin', 'w') as f:
	#	prof.callgrind(f)

