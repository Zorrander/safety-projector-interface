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
    layout_goal.book_adjacent = True
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

    border_projection_goal.request_id = "1"
    border_projection_goal.zone = "table"
    border_projection_goal.position_row = 0
    border_projection_goal.position_col = 1
    border_projection_goal.border.polygon.points = []
    border_projection_goal.border.header.frame_id = "base"
    border_projection_goal.border.header.stamp = rospy.Time.now()
    p = Point32()
    p2 = Point32()
    p.x = 0.5
    p.y = -0.4
    p.z = 0.0
    border_projection_goal.border.polygon.points.append(p);
    p2.x = 0.7
    p2.y = -0.6 
    p2.z = 0.0
    border_projection_goal.border.polygon.points.append(p2)
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

    rospy.spin()

if __name__ == "__main__":
	main()
