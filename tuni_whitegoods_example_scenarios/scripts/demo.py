#!/usr/bin/env python3

import time
import rospy
import actionlib
import moveit_commander
from geometry_msgs.msg import Pose, Point32, PoseStamped

from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import Load
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

from integration.msg import *
from integration.msg import SetSafetyBorderProjectionAction, SetSafetyBorderProjectionGoal
from integration.msg import BookRobotStaticBorderAction, BookRobotStaticBorderGoal
from integration.msg import ReleaseRobotStaticBorderAction, ReleaseRobotStaticBorderGoal
from integration.msg import SetVirtualButtonsProjectionAction, SetVirtualButtonsProjectionGoal
from integration.msg import SetInstructionsProjectionAction, SetInstructionsProjectionGoal
from integration.msg import SetVirtualButtonChangeColorAction
from integration.msg import SetVirtualButtonChangeColorActionGoal, VirtualButtonReference
from integration.srv import ListStaticBordersStatus, ListStaticBordersStatusRequest

from visualization_msgs.msg import Marker
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint

 

class PickAndPlaceNode:
    def __init__(self):
        rospy.init_node("pick_and_place_node")
        moveit_commander.roscpp_initialize([])
        self._setup_variables()
        self._setup_scene()
        self._setup_action_clients()
        self._setup_services()
        self._setup_subscribers() 
        self._log("Loading...")
        self._robot_hand_control_setup()


        constraint = PositionConstraint()
        constraint.link_name = "tool0"
        constraint.header.frame_id = "base_link"

        # Create bounding box
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.6, 1.2, 0.4]  

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.35
        pose.pose.position.y = -0.15
        pose.pose.position.z = 0.3
        pose.pose.orientation.w = 1.0

        constraint.constraint_region.primitives.append(box)
        constraint.constraint_region.primitive_poses.append(pose.pose)
        constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(constraint)
        self.move_group.set_path_constraints(constraints)

        self.publish_constraint_box(center=(0.35, -0.15, 0.3), size=(0.6, 1.2, 0.4))
        reset_needed = rospy.get_param('reset_projections')
        if reset_needed:
            self._project_buttons()
            self._project_borders()

    def _log(self, text):
        log_msg = SetInstructionsProjectionGoal()
        log_msg.title = text
        log_msg.zone = "table"
        self.instructions_client.send_goal(log_msg)
        self.instructions_client.wait_for_result()

    def _setup_variables(self):
        self.border_violated = False
        self.go_pressed = False
        self.stop_pressed = False

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        self.move_group.set_planning_time(10.0)

        ee_link = self.move_group.get_end_effector_link()
        print("End effector link:", ee_link)
        self.slow_rate = rospy.Rate(0.5)
        self.fast_rate = rospy.Rate(2)

        self.slow_rate.sleep()
        

        self.boxes = [[0.30, -0.15, 0.01], [0.30, -0.30, 0.01], [0.30, -0.45, 0.01], [0.30, -0.60, 0.01]]
        self.places = [[0.60, 0.30, 0.01], [0.60, 0.10, 0.01], [0.60, -0.10, 0.01], [0.60, -0.30, 0.01]]
        
        self.boxes_joints = [[-1.9663174788104456, -1.1892793814288538, -0.14740115800966436, -1.5620043913470667, 1.5759005546569824, -3.273930613194601],
                            [-1.8294051329242151, -1.398585621510641, -0.5255878607379358, -1.4904082457171839, 1.5733617544174194, -3.6351380983935755],
                            [-1.569604221974508, -1.686918083821432, -0.7799389998065394, -1.4628809134112757, 1.5720325708389282, -3.908752981816427],
                            [-1.1452620665179651, -2.001375977193014, -0.9438799063311976, -1.5722597281085413, 1.5716131925582886, -4.072701040898458]]

        self.places_joints = [[-1.1522062460528772, -2.0010646025287073, 0.6260181069374084, -1.559077564870016, 1.5789306163787842, -2.483831230794088],
                            [-1.3649166266070765, -1.8442919890033167, 0.34446409344673157, -1.506301228200094, 1.5779128074645996, -2.7651568094836634],
                            [-1.3266099135028284, -1.839625660573141, 0.042819637805223465, -1.5493858496295374, 1.57655930519104, -3.0670631567584437],
                            [-1.1295660177813929, -1.99731952348818, -0.30100948015321904, -1.5918954054461878, 1.5759364366531372, -3.4290831724749964]]

        self.slots = self.boxes + self.places

    def _setup_scene(self):
        def add_box(name, x, y, z, size):
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            self.scene.add_box(name, pose, size=size)

        add_box("table", -0.5, 0.2, -0.02, (1.2, 1.2, 0.02))
        add_box("left_post", -0.13, -0.23, 0.5025, (0.06, 0.03, 1.05))
        add_box("right_post", -0.13, 0.63, 0.5025, (0.06, 0.03, 1.05))
        add_box("crossbar", -0.13, 0.20, 1.03, (0.06, 0.88, 0.02))

    def _setup_action_clients(self):        
        self.instructions_client = self._wait_for_client("/execution/projector_interface/integration/actions/set_instructions_projection", SetInstructionsProjectionAction)
        self.border_client = self._wait_for_client("/execution/projector_interface/integration/actions/set_safety_border_projection", SetSafetyBorderProjectionAction)
        self.border_booking_server = self._wait_for_client("execution/projector_interface/integration/actions/book_robot_static_border", BookRobotStaticBorderAction)
        self.border_releasing_server = self._wait_for_client("execution/projector_interface/integration/actions/release_robot_static_border", ReleaseRobotStaticBorderAction)
        self.project_client = self._wait_for_client(rospy.get_param("button_projection_server_name"), SetVirtualButtonsProjectionAction)
        self.color_client = self._wait_for_client(rospy.get_param("button_color_server_name"), SetVirtualButtonChangeColorAction)

    def _wait_for_client(self, name, action_type):
        client = actionlib.SimpleActionClient(name, action_type)
        client.wait_for_server()
        return client

    def _setup_services(self):
        rospy.wait_for_service("/controller_manager/switch_controller")
        self.switch_controller = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)

        self.load_srv = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        self.play_srv = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        self.connect_srv = rospy.ServiceProxy('/ur_hardware_interface/dashboard/connect', Trigger)
        self.hand_back_control_srv = rospy.ServiceProxy('/ur_hardware_interface/hand_back_control', Trigger)

        self.borders_status_srv = rospy.ServiceProxy('/execution/projector_interface/integration/services/list_static_border_status', ListStaticBordersStatus)

    def _setup_subscribers(self):
        rospy.Subscriber("/execution/projector_interface/integration/topics/safety_border_violation",
                         SafetyBorderViolation, self._border_violation_callback)
        rospy.Subscriber("execution/projector_interface/integration/topics/virtual_button_event_array",
                         VirtualButtonEventArray, self._button_pressed_callback)

    def _border_violation_callback(self, msg):
        self.border_violated = msg.violation_active
        if self.border_violated:
        	self.move_group.stop()

    def _button_pressed_callback(self, msg):
        event = msg.virtual_button_events[0]
        if (event.virtual_button_id == "go_button" and event.event_type == -1):
        	self.go_pressed = True
        	self.stop_pressed = False
        elif (event.virtual_button_id == "stop_button" and event.event_type == -1):
        	self.move_group.stop()
        	self.stop_pressed = True
        	self.go_pressed = False

            	

    def _project_buttons(self):
        self._project_button("go_button", "go", 0.0, 1.0, 0.0, 0.85, -0.6)
        self._project_button("stop_button", "stop", 1.0, 0.0, 0.0, 0.85, -0.4)

    def _project_button(self, request_id, name, r, g, b, x, y):
        goal = SetVirtualButtonsProjectionGoal()
        goal.request_id = request_id
        goal.zone = "table"
        goal.virtual_button = VirtualButtonReference()
        goal.virtual_button.id = name
        goal.virtual_button.zone = "table"
        goal.virtual_button.name = name
        goal.virtual_button.description = f"button {name}"
        goal.virtual_button.text = name.upper()
        goal.virtual_button.button_color.r = r
        goal.virtual_button.button_color.g = g
        goal.virtual_button.button_color.b = b
        goal.virtual_button.button_color.a = 1.0
        goal.virtual_button.text_color.r = 1.0
        goal.virtual_button.text_color.g = 1.0
        goal.virtual_button.text_color.b = 1.0
        goal.virtual_button.text_color.a = 1.0
        goal.virtual_button.center.position.x = x
        goal.virtual_button.center.position.y = y
        goal.virtual_button.center.position.z = 0
        goal.virtual_button.radius = 75.0
        goal.virtual_button.hidden = False

        self.project_client.send_goal(goal)
        self.project_client.wait_for_result()

    def _project_borders(self):
        for box_id, box in enumerate(self.slots):
            goal = SetSafetyBorderProjectionGoal()
            goal.request_id = str(box_id)
            goal.zone = "table"
            goal.position_row = 0
            goal.position_col = 0
            goal.border.header.frame_id = "base"
            goal.border.header.stamp = rospy.Time.now()

            half_width = 0.05
            goal.border.polygon.points = [
                Point32(x=box[0]+half_width, y=box[1]+half_width, z=box[2]),
                Point32(x=box[0]-half_width, y=box[1]+half_width, z=box[2]),
                Point32(x=box[0]-half_width, y=box[1]-half_width, z=box[2]),
                Point32(x=box[0]+half_width, y=box[1]-half_width, z=box[2])
            ]
            goal.border_color.r = 0.0
            goal.border_color.g = 1.0
            goal.border_color.b = 0.0
            goal.border_color.a = 1.0
            goal.is_filled = False
            goal.thickness = 1
            goal.track_violations = True

            self.border_client.send_goal(goal)
            self.border_client.wait_for_result()

    def publish_constraint_box(self, center, size, frame_id="base"):
        marker_pub = rospy.Publisher('/constraint_visualization', Marker, queue_size=1, latch=True)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = ""
        marker.id = 150
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Center of the box
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        marker.pose.orientation.w = 1.0  # no rotation

        # Box size (x, y, z)
        marker.scale.x = size[0]
        marker.scale.y = size[1]
        marker.scale.z = size[2]

        marker.color.a = 0.4  # Transparency
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.lifetime = rospy.Duration(0)  # Keep it visible

        marker_pub.publish(marker)

    def wait(self):
        rospy.sleep(1)

    def _robot_hand_control_setup(self):
        self.wait()
        self._close_gripper()
        self.wait()
        self._open_gripper()
        self.wait()
        self._load_ros_control()

    def _restart_controllers(self):
        req = SwitchControllerRequest()
        req.start_controllers = ['scaled_pos_joint_traj_controller']
        req.strictness = 2
        return self.switch_controller(req)

    def _load_ros_control(self):
        for _ in range(2):
            try:
                self.load_srv("ros_odin_control.urp")
                self.wait()
                self.play_srv()
                self.wait()
                self._restart_controllers()
                self.wait()
                return
            except rospy.ServiceException:
                self.wait()
                self.connect_srv()
                self.wait()

    def _open_gripper(self):
        self._play_gripper_program("test_open_rq85.urp")
        self.wait()

    def _close_gripper(self):
        self._play_gripper_program("test_close_box_rq85.urp")
        self.wait()

    def _play_gripper_program(self, name):
        for _ in range(2):
            try:
                self.load_srv(name)
                break
            except rospy.ServiceException:
                self.fast_rate.sleep()
                self.connect_srv()
                self.fast_rate.sleep()
        self.wait()
        self.play_srv()

    def wait_for_go(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and not self.go_pressed:
            rospy.loginfo("Waiting for 'go' button press...")
            rate.sleep()

    def safe_move_robot(self, pose):
        result = False
        while not result: 
            self.move_group.set_pose_target(pose)
            result = self.move_group.go(wait=True)
            if not result:
                self.move_group.stop()
                self.move_group.clear_pose_targets()
                rospy.logwarn("Motion failed.")
                while (self.stop_pressed or self.border_violated):
                    rospy.logwarn("Stop button pressed or safety violated. Retrying...")
                    self.slow_rate.sleep()
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        self.wait()


    def pick(self, box_id, box, joint_list, book_id):
       
        booking_goal = BookRobotStaticBorderGoal(id=str(book_id))
        print("Booking border - it should turn red")
        self.border_booking_server.send_goal(booking_goal)
        self.wait()
        
        new_pose = Pose()
        new_pose.position.x = -box[0]
        new_pose.position.y = -box[1]
        new_pose.position.z = 0.4
        new_pose.orientation = self.initial_pose.orientation

        self.safe_move_robot(new_pose)

        rospy.loginfo("Reached approach position")

        new_pose.position.z = 0.225
        
        self.safe_move_robot(new_pose)

        self.hand_back_control_srv()

        self._close_gripper()

        self._load_ros_control()

        new_pose.position.z = 0.4

        self.safe_move_robot(new_pose)
        
        releasing_goal = ReleaseRobotStaticBorderGoal(id=str(book_id))
        print("Releasing border - it should turn back to green")
        self.border_releasing_server.send_goal(releasing_goal)
        self.wait()              

    def place(self, box_id, places, joint_list, book_id):

        
        booking_goal = BookRobotStaticBorderGoal(id=str(box_id+4))
        print("Booking border - it should turn red")
        self.border_booking_server.send_goal(booking_goal)
        self.wait()

        new_pose = Pose()
        new_pose.position.x = -places[box_id][0]
        new_pose.position.y = -places[box_id][1]
        new_pose.position.z = 0.4
        new_pose.orientation = self.initial_pose.orientation

        self.safe_move_robot(new_pose)

        rospy.loginfo("Reached approach position")

        new_pose.position.z = 0.215
        
        self.safe_move_robot(new_pose)

        self._open_gripper()
        
        self._load_ros_control()

        new_pose.position.z = 0.4

        self.safe_move_robot(new_pose)

        releasing_goal = ReleaseRobotStaticBorderGoal(id=str(box_id+4))
        print("Releasing border - it should turn back to green")
        self.border_releasing_server.send_goal(releasing_goal)
        self.wait()


    def run(self):
        self._log("Press Go to start.")
        self.wait_for_go()
        self._log("Operations running.")
        rospy.loginfo("Starting pick-and-place operations...")

        joint_goal_clear = self.move_group.get_current_joint_values()
        joint_goal_clear[0] = 0.02
        joint_goal_clear[1] = -0.8538726011859339
        joint_goal_clear[2] = -2.017724339162008
        joint_goal_clear[3] = -1.8451569716082972
        joint_goal_clear[4] = 1.5766551494598389
        joint_goal_clear[5] = -3.136623207722799

        self.move_group.go(joint_goal_clear, wait=True)
        self.move_group.stop()

        self.initial_pose = self.move_group.get_current_pose().pose

        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = "tool0"  
        orientation_constraint.header.frame_id = "base_link"  

        orientation_constraint.orientation = self.initial_pose.orientation

        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1

        orientation_constraint.weight = 0.8

        path_constraints = Constraints()
        path_constraints.orientation_constraints.append(orientation_constraint)

        self.move_group.set_path_constraints(path_constraints)

        OBJECT_FOUND = 2
        EMPTY = 0

        while not rospy.is_shutdown():

            for box_id, box in enumerate(self.boxes):
                status = self.borders_status_srv()
                if status.status_borders[box_id].status == EMPTY:
                    continue
                self.pick(box_id, box, self.boxes_joints, box_id)

                for place_id, place in enumerate(self.places):    
                    status = self.borders_status_srv()
                    if status.status_borders[place_id+4].status == OBJECT_FOUND:
                        continue
                    self.place(box_id, self.places, self.places_joints, place_id+4)
                    break

            self.move_group.go(joint_goal_clear, wait=True)
            self.move_group.stop()

            for place_id, place in enumerate(self.places):
                status = self.borders_status_srv()
                print(status)
                if status.status_borders[place_id+4].status == EMPTY:
                    continue
                self.pick(place_id, place, self.places_joints, place_id+4)


                for box_id, box in enumerate(self.boxes):    
                    status = self.borders_status_srv()
                    if status.status_borders[place_id+4].status == OBJECT_FOUND:
                        continue
                    self.place(place_id, self.boxes, self.boxes_joints, box_id)
                    break

            self.move_group.go(joint_goal_clear, wait=True)
            self.move_group.stop()

if __name__ == "__main__":
    node = PickAndPlaceNode()
    node.run()