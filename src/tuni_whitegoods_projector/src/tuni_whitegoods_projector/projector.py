import cv2
import uuid
import yaml
import json
import rospy
import numpy as np
from pathlib import Path
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from unity_msgs.msg import DataProj
from unity_msgs.msg import ListDataProj
from unity_msgs.msg import ArucoArray
from unity_msgs.msg import ArucoMarker
from unity_msgs.msg import InterfacePOI
from unity_msgs.msg import ElementUI
from unity_msgs.msg import BorderProj

from tuni_whitegoods_projector.button import Button
from tuni_whitegoods_projector.instruction import Instruction
from tuni_whitegoods_projector.ui import InterfaceUI


#class for visualizing with projector
class Projector():
    def __init__(self):
        self.projectors = []
        self.mainCamera = {}
        self.name_f = Path(rospy.get_param("calibration_folder"))
        self.name_folder = self.name_f.parent / "homography" 
        self.is_moving = rospy.get_param("is_moving")
        self.is_init = False
        self.depthIm = None
        self.rgb_img = None
        self.depth2rgb = None
        self.table_space_transform = None
        self.UI_transform = None
        self.static_border = []
        self._screen_size = (1080,1920)
        self._vert_view_size = (1024,1024)
        self._current_joint_values = None
        self.matrix = 0
        self.matrix_moving = 0
        self.matrix_projected = 0
        self._list_interface = [] #InterfaceUI(0,"main","main interface",buttons=None)
        self._cfg = None
        self._button_info = None
        self._robot_carrying_object = False
        self.bridge_interface = CvBridge()
        self.arr_markers = ArucoArray()
        self.marker = ArucoMarker()
        self.prev_tlp = [0,0]
        self.prev_blp = [0,0]
        self.corners_static_table = []
        self.detect_change = True
        self.first = True
        self.sl_dm = None
        self.s_marker = 0
        self.received = False
        self.aruco_changed = False
    
        self.dp_list_pub = rospy.Publisher("/list_dp", ListDataProj, queue_size=1)
        self.pub_poi = rospy.Publisher("/interface_poi/buttons", InterfacePOI, queue_size=1)

        self.viz_test_pub = rospy.Publisher("/test/proj", Image, queue_size=10)
        
        self.init_zones_tmp()
        rospy.loginfo("Projector interface initialized!")

    #get aruco marker on the table
    def callbackArrayMarkers(self,msg):
        self.arr_markers = msg.array_markers
        self.s_marker = len(self.arr_markers)

    #check if aruco marker changed location
    def callback_aruco_change(self,msg):
        self.aruco_changed = msg.data

    def img_cb_once(self, rgb_data, depth_data,depth2RGB_data,subscribers):
        self.depthIm = CvBridge().imgmsg_to_cv2(depth2RGB_data, "passthrough")
        self.rgb_img = CvBridge().imgmsg_to_cv2(rgb_data, "bgr8")
        self.depth2rgb = CvBridge().imgmsg_to_cv2(depth2RGB_data, "passthrough")
	#[sub.sub.unregister() for sub in subscribers]

    #get safety line if any
    def callback_safety_line(self,msg):
        self.sl_dm = CvBridge().imgmsg_to_cv2(msg, "passthrough")
        #self.sl_dm = cv2.cvtColor(tmp,cv2.COLOR_GRAY2RGB)
        self.received = True

    #get dynamic border
    def callback_border(self,msg):
        self.sl_dm = CvBridge().imgmsg_to_cv2(msg.img, "passthrough")
        self.received = True

    #get static borders
    def callback_static_border(self,msg):
        self.static_border = []
        for i in msg.list_borders:
            tmp = BorderProj()
            tmp.img = CvBridge().imgmsg_to_cv2(i.img, "passthrough")
            tmp.zone = i.zone
            self.static_border.append(tmp)
        self.received = True
        print("received static border to display !")
        #cv2.imshow("interface", self.static_border[0].img)
        #cv2.waitKey(1)

    #get a button if the openflow server sends any. But we usually create interface and not just button
    #it was mainly to test openflow
    def callback_button(self, msg):
        b = Button(msg.id,msg.zone,msg.name,msg.description,msg.text,msg.button_color,msg.text_color,msg.center,msg.radius,msg.hidden)
        add_success = False
        if len(self._list_interface) == 0:
            tmp = []
            tmp.append(b)
            print("create default interface")
            ui = InterfaceUI("default",msg.zone,"default_interface","interface by default",tmp)
            self._list_interface.append(ui)
            add_success = True
        else:
            for i in self._list_interface:
                if i.get_zone() == b.get_zone():
                    i.add_button(b)
                    add_success = True
        if not add_success:
            tmp = []
            tmp.append(b)
            print("create new hidden interface")
            ui = InterfaceUI(str(uuid.uuid4()),msg.zone,"default_interface","interface by default",True,tmp)
            self._list_interface.append(ui)
            add_success = True

    def callback_button_color(self, msg):
        for i in self._list_interface:
            if not i.get_hidden():
                i.modify_button_color(msg)

    #get a predefined UI
    def callback_preset_ui(self,msg):
        list_button = []
        for i in msg.virtual_button_references:
            b = Button(i.id,i.zone,i.name,i.description,i.text,i.button_color,i.text_color,i.center,i.radius,i.hidden) 
            list_button.append(b)
        interface = InterfaceUI(msg.resource_id,msg.zone,msg.name,msg.description,list_button)#add hidden
        self._list_interface.append(interface)

    #remove a UI
    def callback_unset(self,msg):
        if msg.data:
            self._list_interface = []
    #get some instructions
    def callback_instruction(self,msg):
        print("got instruction")
        for i in self._list_interface:
            if not i.get_hidden() and msg.zone == i.get_zone():
                print("add instruction")
                inst = Instruction(msg.request_id,msg.zone,msg.target_location,msg.title,msg.title_color,msg.description,msg.description_color,msg.lifetime)
                i.add_instruction(inst)

    #initiate the zones where to display elements by getting the calibration files
    #the calibrations files are homographies
    #this method is the previous way of doing it (when the calibration folder was a mess)
    #
    def init_zones(self):
        name = ""
        with open('/home/altair/odin/src/projector/projector_devices_ur5.json') as f: #data for projectors and the kinect
            prj_json = json.load(f)
            for prj_data in prj_json['projectors']:
                tmp = {}
                tmp['id'] = prj_data['id']
                #tmp['directHom'] = np.load(name+prj_data['homDirect']) 
                #tmp['vertHom'] = np.load(name+prj_data['homFromVert'])#homography for vertical view of RGB image on ground level
                tmp['vertHomTable'] = np.load(name+prj_data['homTableVert'])#homography for vertical view of RGB image on table level
                tmp['hom_proj_static'] = np.load(name+prj_data['hom_proj_static'])
                #print(tmp['hom_proj_static'])
                #tmp['hom_proj_static'] = np.load("/home/altair/odin/src/whitegoods/calibration/homography/hom_proj_static.npy")
                #print(tmp['hom_proj_static'])
                tmp['homDepthProj'] = np.load(name+prj_data['homDepthProj'])#for RGB view to the floor
                tmp['shiftX'] = prj_data['shiftX']#shifting opencv window to the projector
                self.projectors.append(tmp)
            for cam_data in prj_json['cameras']:
                if cam_data['mainCam'] == 1:
                    self.mainCamera['vertHomGround'] = np.load(name+cam_data['vertHomGround'])#homography from normal image to vertical view on ground level
                    self.mainCamera['camRobHom'] = np.load(name+cam_data['camRobHom'])#homography from robot coords to image
                    self.mainCamera['vertHomTable'] = np.load(name+cam_data['vertTable'])#homography from normal image to vertical view on table level
                    self.mainCamera['hom_cam_static'] = np.load(name+cam_data['hom_cam_static'])
                    self.mainCamera['hom_cam_screen_to_proj'] = np.load(name+cam_data['hom_cam_screen_to_proj'])
                    #self.mainCamera['corners_table'] = np.load(name+cam_data['corners'])
        self.find_static_ui_transform()
    
    #this method takes the calibration files and initialize the transform with the homographies
    def init_zones_tmp(self):
        calib_file = self.name_folder / "projection_calibration.yaml"
        with open(calib_file) as file:
         try:
            data = yaml.safe_load(file)
            #print(data)
            for proj in data['proj']:
               tmp = {}
               tmp['id'] = proj['id']
               tmp['zone'] = proj['zone']
               tmp['hom_proj_moving'] = np.load(proj['hom_proj_moving'])
               tmp['hom_proj_static'] = np.load(proj['hom_proj_static'])
               tmp['homDepthProj'] = np.load(proj['homDepthProj'])
               self.projectors.append(tmp)
            #print(tmp)
            for cam in data['cam']:
               tmp = {}
               tmp['id'] = proj['id']
               tmp['zone'] = proj['zone']
               tmp['hom_cam_screen_to_proj'] = np.load(cam['hom_cam_screen_to_proj'])
               tmp['hom_cam_static'] = np.load(cam['hom_cam_static'])
               self.mainCamera = tmp
               #cameras.append(tmp)
            #print(camera)
         except yaml.YAMLError as exception:
            print(exception)
    
        if not self.is_moving:
            self.find_static_ui_transform()
        else:
            #self.matrix = self.mainCamera['hom_cam_static']
            self.matrix_projected = self.mainCamera['hom_cam_screen_to_proj']

    #method called when the display is dynamic like on a moving table.
    #it has to update the transform so it always fit on the table
    def find_dynamic_ui_transform(self):#find homography for the table
        #M = self.mainCamera['vertHomTable']#transform to vertical view
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
        ##if detect_marker == True and t > 0.021 and (diff_tl_x > 1 or diff_tl_y > 1 or diff_br_x > 1 or diff_br_y > 1) :
        #    self.detect_change = True
        #else:
        #    self.detect_change = False

        if (self.aruco_changed or self.first) and self.s_marker > 0:
            self.first = False
            aruco_long_vec = np.array([self.arr_markers[0].width_marker[0],self.arr_markers[0].width_marker[1]])
            aruco_short_vec = np.array([self.arr_markers[0].height_marker[0],self.arr_markers[0].height_marker[1]])
            ##big table
            scale_long = 6.66#compute size of the marker relative to the size of the table to find table corners
            scale_short = 5.0
            tr_point_t = tl_point+aruco_long_vec*scale_long
            bl_point_t = tl_point+aruco_short_vec*scale_short
            br_point_t = tl_point+aruco_short_vec*scale_short+aruco_long_vec*scale_long


            #img_contours = imagefortest #this used for tests, can be deleted later
            pts1 = np.float32([[0,0],[0,self._screen_size[0]],[self._screen_size[1],0]])#corners of the screen
            pts2 = np.float32([tl_point,bl_point_t,tr_point_t])#corners on the image in Kinect
            pts3 = np.float32([tl_point,tr_point_t,br_point_t,bl_point_t])#table corners
            #pts4 = np.float32([[0,0],[self._screen_size[1],0],[0,self._screen_size[0]],[self._screen_size[1],self._screen_size[0]]])
            self.UI_transform = cv2.getAffineTransform(pts1,pts2)
            #print("UI : ",self.UI_transform)
            tmp_transform = np.zeros((3,3))
            tmp_transform[:2,...] = self.UI_transform
            tmp_transform[2,2] = 1
            self.UI_transform = tmp_transform

            pts4 = np.float32([[0,0],[self._screen_size[1],0],[self._screen_size[1],self._screen_size[0]],[0,self._screen_size[0]]])#screen's corners
            self.matrix = cv2.getPerspectiveTransform(pts4,pts3) #fof detection

            #updating values for detection
            self.prev_tlp[0] = tl_point[0]
            self.prev_tlp[1] = tl_point[1]
            self.prev_blp[0] = br_point[0]
            self.prev_blp[1] = br_point[1]

        return
    
    #same as previous method but with a static zone
    def find_static_ui_transform(self): #find transform from screen to table in the camera
        self.matrix = self.mainCamera['hom_cam_static']
        self.matrix_projected = self.mainCamera['hom_cam_screen_to_proj']
        #print("self matrix",self.matrix)
        #print("matrix projected",self.matrix_projected)

    #get buttons position in the RGB camera frame.
    def get_rgb_button_position(self,b,mat):
        px = (mat[0][0]*b[0] + mat[0][1]*b[1] + mat[0][2]) / ((mat[2][0]*b[0] + mat[2][1]*b[1] + mat[2][2]))
        py = (mat[1][0]*b[0] + mat[1][1]*b[1] + mat[1][2]) / ((mat[2][0]*b[0] + mat[2][1]*b[1] + mat[2][2]))
        button = (int(px), int(py))
        
        return button
    
    #display only active interface
    def get_active_interface_image(self):
        interface = None
        found = False
        for i in self._list_interface:
            if not i.get_hidden():
                interface = i.draw()
                found = True
        return interface, found

    #fill message that send the buttons positions in the RGb camera space
    def fillLayoutMessage(self,l_poi):
        interface = None
        for i in self._list_interface:
            if not i.get_hidden():
                interface = i
        if interface != None:
            poi = interface.get_list_buttons()
            if len(poi) > 0:
                #M = self.mainCamera['hom_cam_static']
                #inv = np.linalg.inv(M)
                #inv = np.linalg.inv(self.matrix)
                l_pts = []
                for i in poi:
                    t = i.get_center()
                    #print("t",t)
                    tmp_pt_proj = self.get_rgb_button_position(t,self.matrix_projected)
                    tmp_i = [tmp_pt_proj[0],tmp_pt_proj[1]]
                    tmp_pt_table = self.get_rgb_button_position(tmp_i,self.matrix)
                    tmp_j = [tmp_pt_table[0],tmp_pt_table[1]]
                    #print("tmp_t",tmp_i)
                    l_pts.append(tmp_j)
                #second loop to fill info and send them
                k = 0
                for j in poi:
                    tmp_elem = ElementUI()
                    tmp_elem.id = j._id
                    tmp_elem.elem.x = int(l_pts[k][0])
                    tmp_elem.elem.y = int(l_pts[k][1])
                    k += 1
                    l_poi.poi.append(tmp_elem)
                self.pub_poi.publish(l_poi)
    #run in loop
    def run(self):
        while not rospy.is_shutdown():
            #interface_img = np.zeros((self._screen_size[0], self._screen_size[1], 3), np.uint8)
            tmp = np.zeros((self._screen_size[0], self._screen_size[1], 3), np.uint8)
            interface_tmp, exist = self.get_active_interface_image()
            first = True
            #if the table is moving
            if self.is_moving:
                self.find_dynamic_ui_transform()
            l_poi = InterfacePOI()
            if exist:
                self.fillLayoutMessage(l_poi)

            #M = self.mainCamera['vertHomTable']
            #self._table_homography_pub.publish(M.flatten())
            dp_list = ListDataProj()
            
            #Publish matrix transform and image for each projector
            for proj in self.projectors:#iterate over projectors and display content for each
                #if proj['id'] == 0:
                if exist:
                    dp_interface = DataProj()
                    dp_interface.id = proj['id']
                    dp_interface.transform = tmp.flatten()
                    dp_interface.img = self.bridge_interface.cv2_to_imgmsg(interface_tmp, "bgr8")
                    if not self.is_moving:
                        # tmp = self.matrix  ## seems to work in whirlpool
                        tmp = proj['hom_proj_static']
                        dp_interface.transform = tmp.flatten()
                    else:
                        dp_interface = DataProj()
                        dp_interface.id = proj['id']
                        #tmp = np.matmul(proj['vertHomTable'], self.UI_transform)
                        tmp = np.matmul(proj['hom_proj_moving'], self.UI_transform)
                        dp_interface.transform = tmp.flatten()
                        #tmp = proj['hom_proj_static']
                    dp_list.list_proj.append(dp_interface)

                if self.received:
                    for i in self.static_border:
                        dp_safety = DataProj()
                        dp_safety.id = proj['id']
                        dp_safety.transform = proj['homDepthProj'].flatten()
                        #print(dp_safety.transform)
                        dp_safety.img = self.bridge_interface.cv2_to_imgmsg(i.img, "bgr8")#self.sl_dm
                        dp_list.list_proj.append(dp_safety)
            
                        self.viz_test_pub.publish(dp_safety.img)

            self.dp_list_pub.publish(dp_list)