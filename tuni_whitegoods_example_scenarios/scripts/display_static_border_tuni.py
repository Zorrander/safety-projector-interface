#!/usr/bin/env python3

import time
import rospy
import actionlib
from geometry_msgs.msg import Point32

from integration.msg import *


def main():
    global interaction
    rospy.init_node('test_static_border')

    rospy.loginfo("Loading layout server")
    layout_server_name = "execution/projector_interface/integration/actions/set_layout_static_borders"
    layout_client = actionlib.SimpleActionClient(layout_server_name, SetLayoutStaticBordersAction)
    layout_client.wait_for_server()

    layout_goal = SetLayoutStaticBordersGoal()
    layout_goal.book_adjacent = False
    layout_goal.request_id = "09d-d09d9-fgd"
    layout_goal.safety_factor = 1.0
    layout_goal.size_cols = 1
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
    print("Sending layout goal.")
    layout_client.send_goal(layout_goal)
    layout_client.wait_for_result()

    rospy.loginfo("Loading border server")
    border_server_name = "/execution/projector_interface/integration/actions/set_safety_border_projection"
    border_client = actionlib.SimpleActionClient(border_server_name, SetSafetyBorderProjectionAction)
    border_client.wait_for_server()


    border_projection_goal = SetSafetyBorderProjectionGoal()

    border_projection_goal.request_id = "2"
    border_projection_goal.zone = "shelf"
    border_projection_goal.position_row = 0
    border_projection_goal.position_col = 1
    border_projection_goal.border.polygon.points = []
    border_projection_goal.border.header.frame_id = "base"
    border_projection_goal.border.header.stamp = rospy.Time.now()
    p = Point32()
    p2 = Point32()
    p3 = Point32()
    p4 = Point32()
    p.x = 0.5327723191197576
    p.y = -0.3859754711165183
    p.z = 0.0
    border_projection_goal.border.polygon.points.append(p);

    p2.x = 0.52084058325318
    p2.y = -0.5911370965189771
    p2.z = 0.0
    border_projection_goal.border.polygon.points.append(p2)

    p3.x = 0.31064085560322285
    p3.y = -0.5752820995501067
    p3.z = 0.0
    border_projection_goal.border.polygon.points.append(p3)

    p4.x = 0.32227329926039167
    p4.y = -0.37203207838239905
    p4.z = 0.0
    border_projection_goal.border.polygon.points.append(p4)


    border_projection_goal.border_topic = ""
    border_projection_goal.border_color.r = 0.0
    border_projection_goal.border_color.g = 1.0
    border_projection_goal.border_color.b = 0.0;
    border_projection_goal.border_color.a = 0.0;
    border_projection_goal.is_filled = False;
    border_projection_goal.thickness = 1;
    #border_projection_goal.lifetime.fromNSec(0);
    border_projection_goal.track_violations = True;
    print("Sending border goal.")
    border_client.send_goal(border_projection_goal)
    
    border_client.wait_for_result()



    print("A static border should be visible.")
    print("----------------------------------")
    border_booking_server_name = "execution/projector_interface/integration/actions/book_robot_static_border"
    border_booking_server = actionlib.SimpleActionClient(border_booking_server_name, BookRobotStaticBorderAction)
    border_booking_server.wait_for_server()

    border_releasing_server_name = "execution/projector_interface/integration/actions/release_robot_static_border"
    border_releasing_server = actionlib.SimpleActionClient(border_releasing_server_name, ReleaseRobotStaticBorderAction)
    border_releasing_server.wait_for_server()
    print("Ready to test book/release features. Enter 'b' to book the border 'r' to release it.")
    while not rospy.is_shutdown():
    	action = input("--> ")
    	if action == 'b':
    		border_id = input("border_id--> ")
    		booking_goal = BookRobotStaticBorderGoal(id=border_id)
    		print("Booking border - it should turn red")
    		border_booking_server.send_goal(booking_goal)
    	elif action == 'r':
    		border_id = input("border_id--> ")
    		releasing_goal = ReleaseRobotStaticBorderGoal(id=border_id)
    		print("Releasing border - it should turn back to green")
    		border_releasing_server.send_goal(releasing_goal)

if __name__ == "__main__":
	main()

