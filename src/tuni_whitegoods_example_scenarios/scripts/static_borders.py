#!/usr/bin/env python3

import time
import rospy
import actionlib
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point32
import integration


class Whitegoods_Static_Borders:
    def __init__(self):
        print("Test HMI interface")
        self.go = False 
        self.stop = False 
        self.robot_border = 0

        preset_ui_server_name = "/execution/projector_interface/integration/actions/set_preset_ui_projection"
        self.preset_ui_client = actionlib.SimpleActionClient(preset_ui_server_name, integration.msg.SetPresetUIProjectionAction)
        self.preset_ui_client.wait_for_server()
 
        layout_server_name = "execution/projector_interface/integration/actions/set_layout_static_borders"
        self.layout_client = actionlib.SimpleActionClient(layout_server_name, integration.msg.SetLayoutStaticBordersAction)
        self.layout_client.wait_for_server()

        border_server_name = "/execution/projector_interface/integration/actions/set_safety_border_projection"
        self.border_client = actionlib.SimpleActionClient(border_server_name, integration.msg.SetSafetyBorderProjectionAction)
        self.border_client.wait_for_server()

        interaction_topic = "/execution/projector_interface/integration/topics/virtual_button_event_array"
        self.interaction_sub = rospy.Subscriber(interaction_topic, integration.msg.VirtualButtonEventArray, self.interaction_cb)

        book_robot_static_border_server_name = rospy.get_param("book_robot_static_border_server")
        release_robot_static_border_server_name = rospy.get_param("release_robot_static_border_server")
        book_operator_static_border_server_name = rospy.get_param("book_operator_static_border_server")
        release_operator_static_border_server_name = rospy.get_param("release_operator_static_border_server")

        book_robot_static_border_client = actionlib.SimpleActionClient(book_robot_static_border_server_name, integration.msg.BookRobotStaticBorderAction);
        client_bobook_robot_static_border_clientok_rob.waitForServer()

        release_robot_static_border_client = actionlib.SimpleActionClient(release_robot_static_border_server_name, integration.msg.ReleaseRobotStaticBorderAction);
        release_robot_static_border_client.waitForServer()

        book_operator_static_border_client = actionlib.SimpleActionClient(book_operator_static_border_server_name, integration.msg.BookOperatorStaticBorderAction);
        book_operator_static_border_client.waitForServer()

        release_operator_static_border_client = actionlib.SimpleActionClient(release_operator_static_border_server_name, integration.msg.ReleaseOperatorStaticBorderAction);
        release_operator_static_border_client.waitForServer()

        border_violation_topic = "/execution/projector_interface/integration/topics/safety_border_violation"
        self.border_violation_sub = rospy.Subscriber(border_violation_topic, integration.msg.SafetyBorderViolation, self.border_violation_cb)

    def interaction_cb(self, msg):
        for btn_event in msg.virtual_button_events:
            if btn_event.virtual_button_id == "20":
                if btn_event.event_type == 1:
                    self.stop = True
                    self.go = False
                    # Send moveit stop
                    break
            if btn_event.virtual_button_id == "40":
                if btn_event.event_type == 1:
                    self.go = True
                    self.stop = False
                    # Send moveit go
                    break

    def border_violation_cb(self, msg):
        if msg.request_id == self.robot_border:
            self.stop = True
            self.go = False
            # Send moveit stop

    def init_projection(self):
        # Display preset UI 
        self.project_buttons()
        # Display static borders 
        self.project_static_borders()

    def task_place_object(self):
        #robot.reachHomePosition()
        # check status borders
        # Book a free border
        robot_booking_goal = integration.msg.BookRobotStaticBorderGoal()
        robot_booking_goal.request_id = "29796"
        robot_booking_goal.id = "2"
        book_robot_static_border_client.send_goal(robot_booking_goal)
        book_robot_static_border_client.wait_for_result()
        self.robot_border = 2
        # Place object in it 
        #robot.pickUpObject()
        #robot.placeObjectBorderTwo()
        #robot.reachHomePosition()
        
        # Release 
        robot_release_goal = ReleaseRobotStaticBorderGoal()
        robot_release_goal.request_id = "26968"
        robot_release_goal.id = "2"
        robot_release_goal.status = -1
        release_robot_static_border_client.send_goal(robot_release_goal)
        release_robot_static_border_client.wait_for_result()
        self.robot_border = 0

        # Book it for human 
        operator_booking_goal = BookOperatorStaticBorderGoal()
        operator_booking_goal.request_id = "97345464"
        operator_booking_goal.id = "2"
        book_operator_static_border_client.send_goal(operator_booking_goal)
        book_operator_static_border_client.wait_for_result()

        # Wait and release 
        rospy.Duration(5.0).sleep()

        operator_release_goal = ReleaseOperatorStaticBorderGoal()
        operator_release_goal.request_id = "987964"
        operator_release_goal.id = "2"
        operator_release_goal.status = 0
        release_operator_static_border_client.send_goal(operator_release_goal)
        release_operator_static_border_client.wait_for_result()

     def project_buttons():
          preset_ui_goal = integration.msg.SetPresetUIProjectionGoal()
          
          button_1 = button_2 = button_3 = integration.msg.VirtualButtonReference()
  
          button_1.id = "20";
          button_1.zone = "table";
          button_1.name = "table";
          button_1.description = "button stop";
          button_1.text = "STOP";
          button_1.button_color.r = 1.0;
          button_1.button_color.g = 0.0;
          button_1.button_color.b = 0.0;
          button_1.button_color.a = 0.0;
          button_1.text_color.r = 1.0;
          button_1.text_color.g = 1.0;
          button_1.text_color.b = 1.0;
          button_1.text_color.a = 1.0;
          button_1.center.position.x = 150.0;
          button_1.center.position.y = 880.0;
          button_1.radius = 70.0;
          button_1.hidden = False;

          button_2.id = "40";
          button_2.zone = "table";
          button_2.name = "table";
          button_2.description = "button go";
          button_2.text = "GO";
          button_2.button_color.r = 0.0;
          button_2.button_color.g = 0.0;
          button_2.button_color.b = 1.0;
          button_2.button_color.a = 0.0;
          button_2.text_color.r = 1.0;
          button_2.text_color.g = 1.0;
          button_2.text_color.b = 1.0;
          button_2.text_color.a = 1.0;
          button_2.center.position.x = 840.0;
          button_2.center.position.y = 880.0;
          button_2.radius = 70.0;
          button_2.hidden = False;

          preset_ui_goal.request_id = "98097";
          preset_ui_goal.zone = "table";
          preset_ui_goal.resource_id = "01";
          preset_ui_goal.name = "default ui";
          preset_ui_goal.description = "default ui";
          preset_ui_goal.hidden = false;

          preset_ui_goal.virtual_button_references.append(button_1);
          preset_ui_goal.virtual_button_references.append(button_2);

          self.preset_ui_client.send_goal(preset_ui_goal);
          self.preset_ui_client.wait_for_result();

     def project_static_borders(self):
         layout_goal = SetLayoutStaticBordersGoal()
         border_projection_goal = SetSafetyBorderProjectionGoal()

         layout_goal.book_adjacent = true
         layout_goal.request_id = "09d-d09d9-fgd"
         layout_goal.safety_factor = 1.0
         layout_goal.size_cols = 4
         layout_goal.size_rows = 1
         layout_goal.status_booked.r = 1.0
         layout_goal.status_booked.g = 0.0
         layout_goal.status_booked.b = 0.0
         layout_goal.status_booked.a = 0.0
         layout_goal.status_free.r = 0.0
         layout_goal.status_free.g = 1.0
         layout_goal.status_free.b = 0.0
         layout_goal.status_free.a = 0.0
         layout_goal.status_operator.r = 0.0
         layout_goal.status_operator.g = 0.0
         layout_goal.status_operator.b = 1.0
         layout_goal.status_operator.a = 0.0

         self.layout_client.send_goal(layout_goal)
         self.layout_client.wait_for_result()

         border_projection_goal.request_id = "4"
         border_projection_goal.zone = "shelf"
         border_projection_goal.position_row = 0
         border_projection_goal.position_col = 3
         border_projection_goal.border.polygon.points = []
         border_projection_goal.border.header.frame_id = "base"
         border_projection_goal.border.header.stamp = rospy.Time.now()
         p = Point32
         p.x = 0.5
         p.y = 0.35
         p.z = -0.0
         border_projection_goal.border.polygon.points.append(p)
         p.x = 0.7
         p.y = 0.15 
         p.z = -0.0
         border_projection_goal.border.polygon.points.append(p)
         border_projection_goal.border_topic = ""
         border_projection_goal.border_color.r = 0.0
         border_projection_goal.border_color.g = 1.0
         border_projection_goal.border_color.b = 0.0
         border_projection_goal.border_color.a = 0.0
         border_projection_goal.is_filled = False
         border_projection_goal.thickness = 1
         border_projection_goal.lifetime.fromNSec(0)
         border_projection_goal.track_violations = True

         self.border_client.send_goal(border_projection_goal)
         self.border_client.wait_for_result()
  
         border_projection_goal.request_id = "3"
         border_projection_goal.zone = "shelf"
         border_projection_goal.position_row = 0
         border_projection_goal.position_col = 2
         border_projection_goal.border.polygon.points = []
         border_projection_goal.border.header.frame_id = "base"
         border_projection_goal.border.header.stamp = rospy.Time.now()
         p.x = 0.5
         p.y = 0.1
         p.z = -0.0
         border_projection_goal.border.polygon.points.append(p)
         p.x = 0.7
         p.y = -0.1 
         p.z = -0.0
         border_projection_goal.border.polygon.points.append(p)
         border_projection_goal.border_topic = ""
         border_projection_goal.border_color.r = 0.0
         border_projection_goal.border_color.g = 1.0
         border_projection_goal.border_color.b = 0.0
         border_projection_goal.border_color.a = 0.0
         border_projection_goal.is_filled = False
         border_projection_goal.thickness = 1
         border_projection_goal.lifetime.fromNSec(0)
         border_projection_goal.track_violations = True
          
         self.border_client.send_goal(border_projection_goal)
         self.border_client.wait_for_result();

  
         border_projection_goal.request_id = "2"
         border_projection_goal.zone = "shelf"
         border_projection_goal.position_row = 0
         border_projection_goal.position_col = 1
         border_projection_goal.border.polygon.points = []
         border_projection_goal.border.header.frame_id = "base"
         border_projection_goal.border.header.stamp = rospy.Time.now()
         p.x = 0.5
         p.y = -0.15
         p.z = -0.0
         border_projection_goal.border.polygon.points.append(p)
         p.x = 0.7
         p.y = -0.35 
         p.z = -0.0
         border_projection_goal.border.polygon.points.append(p)
         border_projection_goal.border_topic = ""
         border_projection_goal.border_color.r = 0.0
         border_projection_goal.border_color.g = 1.0
         border_projection_goal.border_color.b = 0.0
         border_projection_goal.border_color.a = 0.0
         border_projection_goal.is_filled = False
         border_projection_goal.thickness = 1
         border_projection_goal.lifetime.fromNSec(0)
         border_projection_goal.track_violations = True
         
         self.border_client.send_goal(border_projection_goal)

         self.border_client.wait_for_result()

         border_projection_goal.request_id = "1"
         border_projection_goal.zone = "shelf"
         border_projection_goal.position_row = 0
         border_projection_goal.position_col = 0
         border_projection_goal.border.polygon.points = []
         border_projection_goal.border.header.frame_id = "base"
         border_projection_goal.border.header.stamp = rospy.Time.now()
         p.x = 0.5
         p.y = -0.4
         p.z = -0.0
         border_projection_goal.border.polygon.points.append(p);
         p.x = 0.7
         p.y = -0.6 
         p.z = -0.0
         border_projection_goal.border.polygon.points.append(p)
         border_projection_goal.border_topic = ""
         border_projection_goal.border_color.r = 0.0
         border_projection_goal.border_color.g = 1.0
         border_projection_goal.border_color.b = 0.0;
         border_projection_goal.border_color.a = 0.0;
         border_projection_goal.is_filled = False;
         border_projection_goal.thickness = 1;
         border_projection_goal.lifetime.fromNSec(0);
         border_projection_goal.track_violations = True;

         self.border_client.send_goal(border_projection_goal)
         self.border_client.wait_for_result()

if __name__ == "__main__":
	openflow = Whitegoods_Static_Borders()
    openflow.init_projection()

    while not rospy.is_shutdown():
        if openflow.interaction:
            openflow.task_place_object()
            break
    

    # init projection
    # place object 

    # check border status service
    # handle border violation 
    # handle button interaction 

    # Pick and place with robot 
    # Pause robot motion 
    # Resume robot motion 


