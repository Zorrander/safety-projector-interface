#!/usr/bin/env python3

import time
import rospy
import actionlib

from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import Load
from geometry_msgs.msg import Point32
import moveit_commander 
from geometry_msgs.msg import Pose

from integration.msg import *
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest


def main():
    moveit_commander.roscpp_initialize([])
    rospy.init_node('test_buttom')

    button_projection_server_name = rospy.get_param("button_projection_server_name") 
    project_client = actionlib.SimpleActionClient(button_projection_server_name, SetVirtualButtonsProjectionAction)
    project_client.wait_for_server()

    button_color_server_name = rospy.get_param("button_color_server_name") 
    color_client = actionlib.SimpleActionClient(button_color_server_name, SetVirtualButtonChangeColorAction)
    color_client.wait_for_server()

    def project_button(x, y):
        ## Project button 
        goal = SetVirtualButtonsProjectionGoal()
        goal.request_id = ""
        goal.zone = "table"
        goal.virtual_button = VirtualButtonReference()
        goal.virtual_button.id = "go";
        goal.virtual_button.zone = "table";
        goal.virtual_button.name = "go";
        goal.virtual_button.description = ""
        goal.virtual_button.text = "o"
        goal.virtual_button.button_color.r = 0.0
        goal.virtual_button.button_color.g = 1.0
        goal.virtual_button.button_color.b = 0.0
        goal.virtual_button.button_color.a = 1.0
        goal.virtual_button.text_color.r = 1.0
        goal.virtual_button.text_color.g = 1.0
        goal.virtual_button.text_color.b = 1.0
        goal.virtual_button.text_color.a = 1.0

        goal.virtual_button.center.position.x = x
        goal.virtual_button.center.position.y = y
        goal.virtual_button.center.position.z = 0

        goal.virtual_button.radius = 40.0
        goal.virtual_button.hidden = False

        # Sends the goal to the action server.
        project_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        project_client.wait_for_result()


    project_button(0.40, 0.250)
    time.sleep(1)
    project_button(0.40, 0.0)
    time.sleep(1)
    project_button(0.40, -0.25)
    time.sleep(1)
    project_button(0.40, -0.50)
    time.sleep(1)

    project_button(0.575, 0.250)
    time.sleep(1)
    project_button(0.575, 0.0)
    time.sleep(1)
    project_button(0.575, -0.25)
    time.sleep(1)
    project_button(0.575, -0.50)
    time.sleep(1)

    project_button(0.75, 0.250)
    time.sleep(1)
    project_button(0.75, 0.0)
    time.sleep(1)
    project_button(0.75, -0.25)

if __name__ == "__main__":
	main()

