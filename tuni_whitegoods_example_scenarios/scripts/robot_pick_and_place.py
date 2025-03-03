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
    rospy.init_node('pick_and_place_test')

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    rospy.sleep(5)

    # Add Table
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "base_link"
    table_pose.pose.position.x = -0.5
    table_pose.pose.position.y = 0.2
    table_pose.pose.position.z = -0.02 # Table height
    scene.add_box("table", table_pose, size=(1.2, 1.2, 0.02))

    # Add Table
    left_post_pose = geometry_msgs.msg.PoseStamped()
    left_post_pose.header.frame_id = "base_link"
    left_post_pose.pose.position.x = -0.13
    left_post_pose.pose.position.y = -0.23
    left_post_pose.pose.position.z = 0.5025 # Table height
    scene.add_box("left_post", left_post_pose, size=(0.06, 0.03, 1.05))

    right_post_pose = geometry_msgs.msg.PoseStamped()
    right_post_pose.header.frame_id = "base_link"
    right_post_pose.pose.position.x = -0.13
    right_post_pose.pose.position.y = 0.63
    right_post_pose.pose.position.z = 0.5025 # Table height
    scene.add_box("right_post", right_post_pose, size=(0.06, 0.03, 1.05))

    crossbar_pose = geometry_msgs.msg.PoseStamped()
    crossbar_pose.header.frame_id = "base_link"
    crossbar_pose.pose.position.x = -0.13
    crossbar_pose.pose.position.y = 0.20
    crossbar_pose.pose.position.z = 1.03 # Table height
    scene.add_box("crossbar", crossbar_pose, size=(0.06, 0.88, 0.02))

    
    rospy.loginfo("Loading border server")
    border_server_name = "/execution/projector_interface/integration/actions/set_safety_border_projection"
    border_client = actionlib.SimpleActionClient(border_server_name, SetSafetyBorderProjectionAction)
    border_client.wait_for_server()

    border_booking_server_name = "execution/projector_interface/integration/actions/book_robot_static_border"
    border_booking_server = actionlib.SimpleActionClient(border_booking_server_name, BookRobotStaticBorderAction)
    border_booking_server.wait_for_server()

    border_releasing_server_name = "execution/projector_interface/integration/actions/release_robot_static_border"
    border_releasing_server = actionlib.SimpleActionClient(border_releasing_server_name, ReleaseRobotStaticBorderAction)
    border_releasing_server.wait_for_server()

    rospy.wait_for_service("/controller_manager/switch_controller")
    switch_controller = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

    box1 = [0.30, -0.15, 0.01]
    box2 = [0.30, -0.30, 0.01]
    box3 = [0.30, -0.45, 0.01]
    box4 = [0.30, -0.60, 0.01]

    place1 = [0.60, 0.30, 0.01]
    place2 = [0.60, 0.0, 0.01]
    place3 = [0.60, -0.30, 0.01]
    place4 = [0.60, -0.60, 0.01]

    boxes = [box1, box2, box3, box4]
    places = [place1, place2, place3, place4]
    slots = [box1, box2, box3, box4, place1, place2, place3, place4]

    border_projection_goal = SetSafetyBorderProjectionGoal()
    p = Point32()
    p2 = Point32()
    p3 = Point32()
    p4 = Point32()
    border_projection_goal.border.polygon.points = []
    border_projection_goal.border.header.frame_id = "base"
    border_projection_goal.border.header.stamp = rospy.Time.now()

    half_width = 0.05

    for box_id, box in enumerate(slots): 
        border_projection_goal.request_id = str(box_id)
        border_projection_goal.zone = "table"
        border_projection_goal.position_row = 0
        border_projection_goal.position_col = 0
        p.x = box[0]+half_width
        p.y = box[1]+half_width
        p.z = box[2]
        border_projection_goal.border.polygon.points.append(p);
        p2.x = box[0]-half_width
        p2.y = box[1]+half_width
        p2.z = box[2]
        border_projection_goal.border.polygon.points.append(p2);
        p3.x = box[0]-half_width
        p3.y = box[1]-half_width
        p3.z = box[2]
        border_projection_goal.border.polygon.points.append(p3);
        p4.x = box[0]+half_width
        p4.y = box[1]-half_width
        p4.z = box[2]
        border_projection_goal.border.polygon.points.append(p4);

        border_projection_goal.border_topic = ""
        border_projection_goal.border_color.r = 0.0
        border_projection_goal.border_color.g = 1.0
        border_projection_goal.border_color.b = 0.0;
        border_projection_goal.border_color.a = 1.0;
        border_projection_goal.is_filled = False;
        border_projection_goal.thickness = 1;
        #border_projection_goal.lifetime.fromNSec(0);
        border_projection_goal.track_violations = True;
        print("Sending border goal.")
        border_client.send_goal(border_projection_goal)
        
        border_client.wait_for_result()

    try:
        rospy.wait_for_service('/ur_hardware_interface/dashboard/connect', 3)
        rospy.wait_for_service('/ur_hardware_interface/dashboard/load_program', 3)
        rospy.wait_for_service('/ur_hardware_interface/dashboard/play', 3)

        load_srv = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        play_srv = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        connect_srv = rospy.ServiceProxy('/ur_hardware_interface/dashboard/connect', Trigger)
        quit_srv = rospy.ServiceProxy('/ur_hardware_interface/dashboard/quit', Trigger)
        hand_back_control_srv = rospy.ServiceProxy('/ur_hardware_interface/hand_back_control', Trigger)
        

        def restart_controllers():

            req = SwitchControllerRequest()

            req.stop_controllers = []
            req.start_controllers = ['scaled_pos_joint_traj_controller']
            req.strictness = 2  # STRICT mode

            response = switch_controller(req)
            if response.ok:
                rospy.loginfo("Controller restarted successfully!")
            else:
                rospy.logwarn("Failed to restart controller.")

        def load_ros_control():
            for i in range(2):
                try:
                    print("attempting to connect")
                    resp = load_srv("ros_odin_control.urp")
                    print(resp)
                    time.sleep(1)  
                    resp = play_srv()
                    time.sleep(1) 
                    print(resp)
                    restart_controllers()
                    time.sleep(0.5)
                    print("Loaded control script")
                except rospy.ServiceException:
                    time.sleep(0.5)  
                    connect_srv()
                    time.sleep(0.5)            

        def open_gripper():
            for i in range(2):
                try:
                    load_srv("test_open_rq85.urp")
                except rospy.ServiceException:
                    time.sleep(0.5)  
                    connect_srv()
                    time.sleep(0.5)
            print("Loaded open script")
            time.sleep(0.5)  
            resp = play_srv()
            print(resp)

        def close_gripper():
            for i in range(2):
                try:
                    load_srv("test_close_box_rq85.urp")
                except rospy.ServiceException:
                    time.sleep(0.5)  
                    connect_srv()
                    time.sleep(0.5)
            print("Loaded close script")
            time.sleep(0.5)  
            resp = play_srv()
            print(resp)
        
        close_gripper()
        time.sleep(1)
        open_gripper()
        time.sleep(1)
        
        load_ros_control()
        time.sleep(1)

        initial_pose = move_group.get_current_pose().pose
        move_group.stop()
        

        while not rospy.is_shutdown():

            joint_goal_clear = move_group.get_current_joint_values()
            print(joint_goal_clear)
            joint_goal_clear[0] = 0.02
            joint_goal_clear[1] = -0.8538726011859339
            joint_goal_clear[2] = -2.017724339162008
            joint_goal_clear[3] = -1.8451569716082972
            joint_goal_clear[4] = 1.5766551494598389
            joint_goal_clear[5] = -3.136623207722799

            for box_id, box in enumerate(boxes):

                move_group.go(joint_goal_clear, wait=True)
                move_group.stop()

                booking_goal = BookRobotStaticBorderGoal(id=str(box_id))
                print("Booking border - it should turn red")
                border_booking_server.send_goal(booking_goal)
                time.sleep(1)

                new_pose = Pose()
                new_pose.position.x = -box[0]
                new_pose.position.y = -box[1]
                new_pose.position.z = 0.4
                new_pose.orientation = initial_pose.orientation

                move_group.set_pose_target(new_pose)
                move_group.go(wait=True)
                move_group.stop()
                move_group.clear_pose_targets()

                new_pose.position.z = 0.225
                
                move_group.set_pose_target(new_pose)
                move_group.go(wait=True)
                move_group.stop()
                move_group.clear_pose_targets()
                
                #hand_back_control_srv()

                close_gripper()
                time.sleep(1)

                load_ros_control()
                time.sleep(1)
                
                releasing_goal = ReleaseRobotStaticBorderGoal(id=str(box_id))
                print("Releasing border - it should turn back to green")
                border_releasing_server.send_goal(releasing_goal)

                booking_goal = BookRobotStaticBorderGoal(id=str(box_id+4))
                print("Booking border - it should turn red")
                border_booking_server.send_goal(booking_goal)
                time.sleep(1)
                
                new_pose = Pose()
                new_pose.position.x = -places[box_id][0]
                new_pose.position.y = -places[box_id][1]
                new_pose.position.z = 0.4
                new_pose.orientation = initial_pose.orientation

                move_group.set_pose_target(new_pose)
                move_group.go(wait=True)
                move_group.stop()
                move_group.clear_pose_targets()

                new_pose.position.z = 0.225
                
                move_group.set_pose_target(new_pose)
                move_group.go(wait=True)
                move_group.stop()
                move_group.clear_pose_targets()

                open_gripper()
                time.sleep(1)

                load_ros_control()
                time.sleep(1)

                releasing_goal = ReleaseRobotStaticBorderGoal(id=str(box_id+4))
                print("Releasing border - it should turn back to green")
                border_releasing_server.send_goal(releasing_goal)


            for place_id, place in enumerate(places):
                move_group.go(joint_goal_clear, wait=True)
                move_group.stop()

                booking_goal = BookRobotStaticBorderGoal(id=str(place_id+4))
                print("Booking border - it should turn red")
                border_booking_server.send_goal(booking_goal)
                time.sleep(1)


                new_pose = Pose()
                new_pose.position.x = -place[0]
                new_pose.position.y = -place[1]
                new_pose.position.z = 0.4
                new_pose.orientation = initial_pose.orientation

                move_group.set_pose_target(new_pose)
                move_group.go(wait=True)
                move_group.stop()
                move_group.clear_pose_targets()

                new_pose.position.z = 0.225
                
                move_group.set_pose_target(new_pose)
                move_group.go(wait=True)
                move_group.stop()
                move_group.clear_pose_targets()
                
                #hand_back_control_srv()

                close_gripper()
                time.sleep(1)

                load_ros_control()
                time.sleep(1)
                
                releasing_goal = ReleaseRobotStaticBorderGoal(id=str(place_id+4))
                print("Releasing border - it should turn back to green")
                border_releasing_server.send_goal(releasing_goal)

                booking_goal = BookRobotStaticBorderGoal(id=str(place_id))
                print("Booking border - it should turn red")
                border_booking_server.send_goal(booking_goal)
                time.sleep(1)
                
                new_pose = Pose()
                new_pose.position.x = -boxes[place_id][0]
                new_pose.position.y = -boxes[place_id][1]
                new_pose.position.z = 0.4
                new_pose.orientation = initial_pose.orientation

                move_group.set_pose_target(new_pose)
                move_group.go(wait=True)
                move_group.stop()
                move_group.clear_pose_targets()

                new_pose.position.z = 0.225
                
                move_group.set_pose_target(new_pose)
                move_group.go(wait=True)
                move_group.stop()
                move_group.clear_pose_targets()

                open_gripper()
                time.sleep(1)

                load_ros_control()
                time.sleep(1)

                releasing_goal = ReleaseRobotStaticBorderGoal(id=str(place_id))
                print("Releasing border - it should turn back to green")
                border_releasing_server.send_goal(releasing_goal)

    

    except rospy.ROSException as e:
        rospy.logerr("Service call failed or timed out: %s", str(e))



if __name__ == "__main__":
	main()

