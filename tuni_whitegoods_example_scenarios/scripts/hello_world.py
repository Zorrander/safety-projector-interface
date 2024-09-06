#!/usr/bin/env python3

import time
import rospy
import actionlib

from std_msgs.msg import ColorRGBA
from integration.msg import VirtualButtonReference, VirtualButtonEventArray
from integration.msg import SetVirtualButtonsProjectionAction, SetVirtualButtonsProjectionGoal, SetVirtualButtonChangeColorAction, SetVirtualButtonChangeColorGoal


interaction = False 


def interaction_cb(msg):
    global interaction
    for btn_event in msg.virtual_button_events:
        if btn_event.virtual_button_id == "40":
            if btn_event.event_type == 1:
                interaction = True
                break 

def main():
    global interaction
    rospy.init_node('hello_world')

    button_projection_server_name = rospy.get_param("button_projection_server_name") 
    project_client = actionlib.SimpleActionClient(button_projection_server_name, SetVirtualButtonsProjectionAction)
    project_client.wait_for_server()

    button_color_server_name = rospy.get_param("button_color_server_name") 
    color_client = actionlib.SimpleActionClient(button_color_server_name, SetVirtualButtonChangeColorAction)
    color_client.wait_for_server()

    interaction_topic = "/execution/projector_interface/integration/topics/virtual_button_event_array"
    interaction_sub = rospy.Subscriber(interaction_topic, VirtualButtonEventArray, interaction_cb)
    ## Project button 
    goal = SetVirtualButtonsProjectionGoal()
    goal.request_id = "go_button"
    goal.zone = "shelf"
    goal.virtual_button = VirtualButtonReference()
    goal.virtual_button.id = "40";
    goal.virtual_button.zone = "shelf";
    goal.virtual_button.name = "go";
    goal.virtual_button.description = "button go"
    goal.virtual_button.text = "GO"
    goal.virtual_button.button_color.r = 0.0
    goal.virtual_button.button_color.g = 0.0
    goal.virtual_button.button_color.b = 1.0
    goal.virtual_button.button_color.a = 0.0
    goal.virtual_button.text_color.r = 1.0
    goal.virtual_button.text_color.g = 1.0
    goal.virtual_button.text_color.b = 1.0
    goal.virtual_button.text_color.a = 1.0
    goal.virtual_button.center.position.x = 0.6347894913928517-0.075
    goal.virtual_button.center.position.y = 1.4488399414350095-0.05
    goal.virtual_button.center.position.z = 0.4233126879551239
    goal.virtual_button.radius = 30.0
    goal.virtual_button.hidden = False

    # Sends the goal to the action server.
    project_client.send_goal(goal)

    # Waits for the server to finish performing the action.
    project_client.wait_for_result()

    ## Project button 2

    goal = SetVirtualButtonsProjectionGoal()
    goal.request_id = "stop_button"
    goal.zone = "shelf"
    goal.virtual_button = VirtualButtonReference()
    goal.virtual_button.id = "stop";
    goal.virtual_button.zone = "shelf";
    goal.virtual_button.name = "go";
    goal.virtual_button.description = "button go"
    goal.virtual_button.text = "STOP"
    goal.virtual_button.button_color.r = 1.0
    goal.virtual_button.button_color.g = 0.0
    goal.virtual_button.button_color.b = 0.0
    goal.virtual_button.button_color.a = 0.0
    goal.virtual_button.text_color.r = 1.0
    goal.virtual_button.text_color.g = 1.0
    goal.virtual_button.text_color.b = 1.0
    goal.virtual_button.text_color.a = 1.0
    goal.virtual_button.center.position.x = 0.45509882460172196
    goal.virtual_button.center.position.y = 1.2754568319698545+0.05
    goal.virtual_button.center.position.z = 0.3183901015238977
    goal.virtual_button.radius = 30.0
    goal.virtual_button.hidden = False

    # Sends the goal to the action server.
    project_client.send_goal(goal)

    # Waits for the server to finish performing the action.
    project_client.wait_for_result()


    color_goal = SetVirtualButtonChangeColorGoal()
    color_goal.request_id = "go_button_color"
    color_goal.resource_id = "40" 
    color_goal.button_color = ColorRGBA()
    color_goal.button_color.r = 0.0
    color_goal.button_color.g = 1.0
    color_goal.button_color.b = 0.0
    color_goal.button_color.a = 0.0

    # Change color
    while not rospy.is_shutdown():
        if interaction:
            print("Hello world!")
            #color_client.send_goal(color_goal)
            #color_client.wait_for_result()
            break


if __name__ == "__main__":
	main()

