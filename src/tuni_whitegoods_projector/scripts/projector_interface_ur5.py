#!/usr/bin/env python3
#This code draws an interface or elements that comes from the openflow server.
#It also project everything that needs to be projected by associating each element to be projected (interface, instructions, borders...) with an homography
#

import rospy

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
    
    sub_preset_ui = rospy.Subscriber("/interfaceUI/openflow/new_interface", ProjectorUI, proj.callback_preset_ui)
    
    sub_static_border_proj = rospy.Subscriber("/projector_interface/display_static_border", LBorder, proj.callback_static_border)

    sub_unset_proj = rospy.Subscriber("/interfaceUI/openflow/unset_projection", Bool, proj.callback_unset)
    sub_instruction = rospy.Subscriber("/interfaceUI/openflow/set_instruction", Instructions, proj.callback_instruction)

    proj.run()


if __name__ == "__main__":
	main()

