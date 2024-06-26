#!/usr/bin/env python3
#This code draws an interface or elements that comes from the openflow server.
#It also project everything that needs to be projected by associating each element to be projected (interface, instructions, borders...) with an homography
#

import rospy

import message_filters

from sensor_msgs.msg import Image

from unity_msgs.msg import ArucoArray
from unity_msgs.msg import BorderProj
from unity_msgs.msg import Instructions
from unity_msgs.msg import LBorder

from std_msgs.msg import Bool

from integration.msg import VirtualButtonReference
from integration.msg import ProjectorUI

from tuni_whitegoods_projector.projector import Projector

def main():
    rospy.init_node('projection_system')

    proj = Projector()

    sub_button = rospy.Subscriber("/interfaceUI/openflow/new_button", VirtualButtonReference, proj.callback_button)
    sub_button_color = rospy.Subscriber("/interfaceUI/openflow/change_button_color", VirtualButtonReference, proj.callback_button_color)
    marker_array_sub = rospy.Subscriber("/aruco_markers_warped", ArucoArray, proj.callbackArrayMarkers)
    aruco_change = rospy.Subscriber("/aruco_markers_warped/changes", Bool, proj.callback_aruco_change)
    sub_safety_line = rospy.Subscriber("/safety_line/line_proj", Image, proj.callback_safety_line)
    sub_preset_ui = rospy.Subscriber("/interfaceUI/openflow/new_interface", ProjectorUI, proj.callback_preset_ui)
    sub_border_proj = rospy.Subscriber("/projector_interface/display_dynamic_border", BorderProj, proj.callback_border)
    sub_static_border_proj = rospy.Subscriber("/projector_interface/display_static_border", LBorder, proj.callback_static_border)
    sub_unset_proj = rospy.Subscriber("/interfaceUI/openflow/unset_projection", Bool, proj.callback_unset)
    sub_instruction = rospy.Subscriber("/interfaceUI/openflow/set_instruction", Instructions, proj.callback_instruction)

    rgb_sub = message_filters.Subscriber("/master/rgb/image_raw", Image)
    depth_sub = message_filters.Subscriber("/master/depth/image_raw", Image)
    depth2RGB_sub = message_filters.Subscriber("/master/depth_to_rgb/image_raw", Image)
    im_subs = [rgb_sub, depth_sub, depth2RGB_sub]
    ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, depth2RGB_sub], 1, 2)
    ts.registerCallback(proj.img_cb_once, im_subs)
  
    proj.run()


if __name__ == "__main__":
	main()

