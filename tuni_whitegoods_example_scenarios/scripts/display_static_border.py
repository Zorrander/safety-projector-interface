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
    p = Point32()
    p2 = Point32()
    p3 = Point32()
    p4 = Point32()
    border_projection_goal.border.polygon.points = []
    border_projection_goal.border.header.frame_id = "base"
    border_projection_goal.border.header.stamp = rospy.Time.now()
    
    '''
    border_projection_goal.request_id = "2"
    border_projection_goal.zone = "shelf"
    border_projection_goal.position_row = 0
    border_projection_goal.position_col = 1
   
    p.x = 0.2470192313637794
    p.y = 1.0076322789399699
    p.z = 0.1765557378920295
    border_projection_goal.border.polygon.points.append(p);
    p2.x =  0.3654504213040404
    p2.y = 0.8976344283797455
    p2.z = 0.17768913076960136
    border_projection_goal.border.polygon.points.append(p2);
    p3.x = 0.31540976438689644
    p3.y = 0.8434376995645345
    p3.z = 0.1466768192627892
    border_projection_goal.border.polygon.points.append(p3);
    p4.x = 0.1969785744466354
    p4.y = 0.9534355501247589
    p4.z = 0.1455434263852171
    border_projection_goal.border.polygon.points.append(p4);

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
    '''
    '''
    1st point
    '''
    x_offset = -0.1
    offset = 0.0

    border_projection_goal.request_id = "1"
    border_projection_goal.zone = "shelf"
    border_projection_goal.position_row = 0
    border_projection_goal.position_col = 1
    p.x = 0.2411861139422537-x_offset
    p.y = 0.9989556644497138+offset
    p.z = 0.1722252372473725
    border_projection_goal.border.polygon.points.append(p);
    p2.x = 0.3584447315720875-x_offset
    p2.y =  0.890046888767809+offset
    p2.z = 0.17334740854466868
    border_projection_goal.border.polygon.points.append(p2);
    p3.x = 0.18630491874250504-x_offset
    p3.y = 0.7036101925095353+offset
    p3.z = 0.06666508606767474
    border_projection_goal.border.polygon.points.append(p3);
    p4.x = 0.06904630111267124-x_offset
    p4.y = 0.8125189681914401+offset
    p4.z = 0.06554291477037855
    border_projection_goal.border.polygon.points.append(p4);

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

    '''
    2nd point
    '''
    border_projection_goal.request_id = "2"
    border_projection_goal.zone = "shelf"
    border_projection_goal.position_row = 0
    border_projection_goal.position_col = 2
    p.x = 0.4170740403870044-x_offset
    p.y = 0.8355925009268566+offset
    p.z = 0.17390849419331667
    border_projection_goal.border.polygon.points.append(p);
    p2.x = 0.5343326470988294-x_offset
    p2.y = 0.7266837353855026+offset
    p2.z = 0.17503066538612688
    border_projection_goal.border.polygon.points.append(p2);
    p3.x = 0.3621928342692469-x_offset
    p3.y = 0.5402470391272289+offset
    p3.z = 0.06834834290913294
    border_projection_goal.border.polygon.points.append(p3);
    p4.x = 0.24493422755742195-x_offset
    p4.y = 0.6491558046685828+offset
    p4.z = 0.06722617171632272
    border_projection_goal.border.polygon.points.append(p4);

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


    '''
    3rd point
    '''
    border_projection_goal.request_id = "3"
    border_projection_goal.zone = "shelf"
    border_projection_goal.position_row = 0
    border_projection_goal.position_col = 3
    p.x = 0.5929619559137462-x_offset
    p.y = 0.6722293475445502+offset
    p.z = 0.17559175103477487
    border_projection_goal.border.polygon.points.append(p);
    p2.x =0.7102205626255713-x_offset
    p2.y =0.5633205820031961+offset
    p2.z =0.17671392222758509
    border_projection_goal.border.polygon.points.append(p2);
    p3.x = 0.5380807497959887-x_offset
    p3.y = 0.37688388574492243+offset
    p3.z = 0.07003159975059114
    border_projection_goal.border.polygon.points.append(p3);
    p4.x = 0.4208221430841638-x_offset
    p4.y = 0.4857926512862765+offset
    p4.z = 0.06890942855778093
    border_projection_goal.border.polygon.points.append(p4);

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

    '''
    4th point
    
    border_projection_goal.request_id = "4"
    border_projection_goal.zone = "shelf"
    border_projection_goal.position_row = 0
    border_projection_goal.position_col = 4
    p.x = 0.7688498550634749
    p.y = 0.50886620937307
    p.z = 0.17727500771950422
    border_projection_goal.border.polygon.points.append(p);
    p2.x =0.8861084617753
    p2.y =0.399957443831716
    p2.z =0.17839717891231444
    border_projection_goal.border.polygon.points.append(p2);
    p3.x = 0.7139686489457175
    p3.y = 0.21352074757344236
    p3.z = 0.0717148564353205
    border_projection_goal.border.polygon.points.append(p3);
    p4.x = 0.5967100422338925
    p4.y = 0.32242951311479634
    p4.z = 0.07059268524251028
    border_projection_goal.border.polygon.points.append(p4);

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
    '''

    '''
    5th point
    
    border_projection_goal.request_id = "5"
    border_projection_goal.zone = "shelf"
    border_projection_goal.position_row = 0
    border_projection_goal.position_col = 5
    p.x = 0.9447377760492213
    p.y = 0.3455030509204882
    p.z = 0.17895826461320552
    border_projection_goal.border.polygon.points.append(p);
    p2.x = 1.061996404597064
    p2.y = 0.23659426509803244
    p2.z = 0.1800804360149877
    border_projection_goal.border.polygon.points.append(p2);
    p3.x = 0.8898565917674813
    p3.y = 0.05015756883975886
    p3.z = 0.07339811353799353
    border_projection_goal.border.polygon.points.append(p3);
    p4.x = 0.7725979632196387
    p4.y = 0.15906635466221447
    p4.z = 0.07227594213621158
    border_projection_goal.border.polygon.points.append(p4);

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
    '''

    '''
    6th point
    
    border_projection_goal.request_id = "6"
    border_projection_goal.zone = "shelf"
    border_projection_goal.position_row = 0
    border_projection_goal.position_col = 6
    p.x = 1.12062567519895
    p.y = 0.18213991274900804
    p.z = 0.18064152129793487
    border_projection_goal.border.polygon.points.append(p);
    p2.x = 1.237884260074757
    p2.y = 0.07323116748875558
    p2.z = 0.18176369228177314
    border_projection_goal.border.polygon.points.append(p2);
    p3.x = 1.0657444472451747
    p3.y = -0.11320552876951795
    p3.z = 0.0750813698047792
    border_projection_goal.border.polygon.points.append(p3);
    p4.x = 0.9484858623693674
    p4.y = -0.004296783509265767
    p4.z = 0.07395919882094093
    border_projection_goal.border.polygon.points.append(p4);

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

    '''

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
