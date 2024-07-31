import cv2
import uuid
import yaml
import json
import rospy
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs.tf2_geometry_msgs

from visualization_msgs.msg import Marker

from geometry_msgs.msg import PoseStamped
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

from tuni_whitegoods_msgs.srv import TransformRobotCameraCoordinates
from tuni_whitegoods_msgs.srv import Transform3DToPixel


#class for visualizing with projector
class Projector():
    def __init__(self):
        self.is_moving = rospy.get_param("is_moving")

        self.static_border = []
        self._list_interface = [] #InterfaceUI(0,"main","main interface",buttons=None)

        self.bridge_interface = CvBridge()
        self.arr_markers = ArucoArray()

        self.transform_world_coordinates = rospy.ServiceProxy('transform_world_coordinates_frame', TransformRobotCameraCoordinates)
        self.project_3D_to_pixel = rospy.ServiceProxy('transform_3D_to_pixel', Transform3DToPixel)

        self.object_detection_sub = rospy.Subscriber("odin/visualization/object_detection", Image, self.viz_callback);
        self.object_detection_pub = rospy.Publisher('odin/visualization/scene_detection', Image, queue_size=10)
        self.test = rospy.Publisher('odin/visualization/test', Image, queue_size=10)

        self.dp_list_pub = rospy.Publisher("/list_dp", ListDataProj, queue_size=1)

        self.vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        self.received = False

        rospy.loginfo("Projector interface initialized!")

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

    def viz_callback(self, msg):
        self.cv_image = self.bridge_interface.imgmsg_to_cv2(msg, "passthrough")
        # Convert read-only image to writable image if necessary
        if not self.cv_image.flags['WRITEABLE']:
            self.cv_image = np.copy(self.cv_image)

    def callback_button(self, msg):
        # Create Rviz marker 
        marker = Marker()
        marker.header.frame_id = "base"
        marker.header.stamp = rospy.Time.now()
        marker.id = int(msg.id)
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = msg.center.position.x
        marker.pose.position.y = msg.center.position.y
        # Set the dimensions of the cylinder
        marker.scale.x = msg.radius / 500 
        marker.scale.y = msg.radius / 500
        marker.scale.z = 0.01  # Height of the cylinder
        # Set the color of the cylinder
        marker.color.r = msg.button_color.r;
        marker.color.g = msg.button_color.g;
        marker.color.b = msg.button_color.b;
        marker.color.a = 1.0;
        self.vis_pub.publish( marker );

        # Create the input pose stamped message
        in_point_stamped = PoseStamped()
        in_point_stamped.header.frame_id = "base"
        in_point_stamped.header.stamp = rospy.Time(0)
        in_point_stamped.pose.position.x = msg.center.position.x
        in_point_stamped.pose.position.y = msg.center.position.y

        # Perform the transformation
        cam_coordinates = self.transform_world_coordinates(in_point_stamped, "rgb_camera_link")

        # Project to 2D image coordinates
        pixel_coordinates = self.project_3D_to_pixel(cam_coordinates.out_point_stamped.pose.position.x, cam_coordinates.out_point_stamped.pose.position.y, cam_coordinates.out_point_stamped.pose.position.z)
        center_cam_point_x = pixel_coordinates.u
        center_cam_point_y = pixel_coordinates.v
        cv2.circle(self.cv_image, (center_cam_point_x, center_cam_point_y), int(msg.radius), (255, 255, 255), 2)
        print("FINAL COORDINATE", center_cam_point_x, center_cam_point_y)
            

        b = Button(msg.id,msg.zone,msg.name,msg.description,msg.text,msg.button_color,msg.text_color, (center_cam_point_x, center_cam_point_y),msg.radius,msg.hidden)
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

        self.object_detection_pub.publish(self.bridge_interface.cv2_to_imgmsg(self.cv_image, "bgr8"))

    def callback_button_color(self, msg):
        print("CHANGING COLOR")
        print("""


            """)
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

    
    #display only active interface
    def get_active_interface_image(self):
        interface = None
        found = False
        for i in self._list_interface:
            if not i.get_hidden():
                interface = i.draw()
                found = True
        return interface, found

    #run in loop
    def run(self):
        while not rospy.is_shutdown():
            interface_tmp, exist = self.get_active_interface_image()

            dp_list = ListDataProj()
            
            if exist:
                dp_interface = DataProj()
                dp_interface.img = self.bridge_interface.cv2_to_imgmsg(interface_tmp, "bgr8")
                dp_list.list_proj.append(dp_interface)
            if self.received:
                for i in self.static_border:
                    dp_safety = DataProj()
                    dp_safety.img = self.bridge_interface.cv2_to_imgmsg(i.img, "bgr8")
                    self.test.publish(dp_safety.img)
                    dp_list.list_proj.append(dp_safety)
            self.dp_list_pub.publish(dp_list)