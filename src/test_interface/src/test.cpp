#include <iostream>
#include <ros/ros.h>
//#include "use_class/NumberCounter.hpp"
#include "border/DynamicBorder.hpp"
#include <integration/SetPresetUIProjectionActionGoal.h>
#include <integration/SetPresetUIProjectionAction.h>
#include <integration/SetInstructionsProjectionAction.h>
#include <integration/SetInstructionsProjectionActionGoal.h>
#include <integration/SetSafetyBorderProjectionAction.h>
#include <integration/SetSafetyBorderProjectionActionGoal.h>
#include <integration/SetLayoutStaticBordersAction.h>
#include <integration/SetLayoutStaticBordersGoal.h>
#include <integration/BookRobotStaticBorderAction.h>
#include <integration/BookRobotStaticBorderGoal.h>
#include <integration/ReleaseRobotStaticBorderAction.h>
#include <integration/ReleaseRobotStaticBorderGoal.h>
#include <integration/BookOperatorStaticBorderAction.h>
#include <integration/BookOperatorStaticBorderGoal.h>
#include <integration/ReleaseOperatorStaticBorderAction.h>
#include <integration/ReleaseOperatorStaticBorderGoal.h>
#include <integration/VirtualButtonReference.h>
#include <integration/MoveJointsAction.h>
#include <integration/MoveJointsGoal.h>
#include <integration/ControlOpenCloseToolAction.h>
#include <integration/ControlOpenCloseToolGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <ur_dashboard_msgs/Load.h>
#include <string.h> 


using namespace integration;

typedef actionlib::SimpleActionClient<SetPresetUIProjectionAction> Client;
typedef actionlib::SimpleActionClient<SetInstructionsProjectionAction> Client_instruct;
typedef actionlib::SimpleActionClient<SetSafetyBorderProjectionAction> Client_static_border;
typedef actionlib::SimpleActionClient<SetLayoutStaticBordersAction> Client_layout;
typedef actionlib::SimpleActionClient<BookRobotStaticBorderAction> Client_book_robot;
typedef actionlib::SimpleActionClient<ReleaseRobotStaticBorderAction> Client_release_robot;
typedef actionlib::SimpleActionClient<BookOperatorStaticBorderAction> Client_book_operator;
typedef actionlib::SimpleActionClient<ReleaseOperatorStaticBorderAction> Client_release_operator;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Trajectory_action;
typedef actionlib::SimpleActionClient<MoveJointsAction> Client_move_robot;
typedef actionlib::SimpleActionClient<ControlOpenCloseToolAction> Client_ctrl_gripper;


class UR5
{
    private:
      ros::NodeHandle nh_;
      ros::Publisher pub_command;
      integration::MoveJointsGoal goal_ur5;
      ros::ServiceClient client_load;
      ros::ServiceClient client_play;
      ur_dashboard_msgs::Load srv_req;
      std_srvs::Trigger srv_play;
      Trajectory_action *trajectory_ur5;
      Client_move_robot *move_joints;
      Client_ctrl_gripper *ctrl_gripper;
      integration::ControlOpenCloseToolGoal goal_gripper;
      bool incoming;
    public:
      UR5(ros::NodeHandle *nh_)
      {
        client_load = nh_->serviceClient<ur_dashboard_msgs::Load>("/ur_hardware_interface/dashboard/load_program");
        client_play = nh_->serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/play");

        pub_command = nh_->advertise<std_msgs::String> ("/ur_hardware_interface/script_command", 1);
        goal_ur5.trajectory.joint_names.resize(6);
        goal_ur5.trajectory.points.resize(1);
        goal_ur5.trajectory.joint_names.push_back("elbow_joint");
        goal_ur5.trajectory.joint_names.push_back("shoulder_lift_joint");
        goal_ur5.trajectory.joint_names.push_back("shoulder_pan_joint");
        goal_ur5.trajectory.joint_names.push_back("wrist_1_joint");
        goal_ur5.trajectory.joint_names.push_back("wrist_2_joint");
        goal_ur5.trajectory.joint_names.push_back("wrist_3_joint");
        incoming = true;
        /*move_joints = new Client_move_robot("test",true);
        while(!move_joints->waitForServer(ros::Duration(5.0)))
        {
          ROS_INFO("Waiting for the move_joints server");
        }
        ctrl_gripper = new Client_ctrl_gripper("test_gripper",true);
        while(!ctrl_gripper->waitForServer(ros::Duration(5.0)))
        {
          ROS_INFO("Waiting for the move_joints server");
        }*/

      }
      ~UR5()
      {
        delete trajectory_ur5;
      }

      bool getIncoming()
      {
        return incoming;
      }

      void closeGripper()
      {
        srv_req.request.filename = "cl.urp";
        if (client_load.call(srv_req))
        {
          std::cout<<srv_req.response.answer<<"\n";
        }
        else
        {
          ROS_ERROR("Failed to call service load");
          //return 1;
        }
        if (client_play.call(srv_play))
        {
          std::cout<<srv_play.response.message<<"\n";
        }
        else
        {
          ROS_ERROR("Failed to call service play");
          //return 1;
        }
        std::cout<<"Test continue 2 \n";
        ros::Duration(2.5).sleep();
      }

      void openGripper()
      {
        srv_req.request.filename = "open.urp";
        if (client_load.call(srv_req))
        {
          std::cout<<srv_req.response.answer<<"\n";
        }
        else
        {
          ROS_ERROR("Failed to call service load");
          //return 1;
        }
        if (client_play.call(srv_play))
        {
          std::cout<<srv_play.response.message<<"\n";
        }
        else
        {
          ROS_ERROR("Failed to call service play");
          //return 1;
        }
        std::cout<<"Test continue 2 \n";
        ros::Duration(2.5).sleep();
      }

      void loadRosControl()
      {
        srv_req.request.filename = "ros_control.urp";
        if (client_load.call(srv_req))
        {
          std::cout<<srv_req.response.answer<<"\n";
        }
        else
        {
          ROS_ERROR("Failed to call service load");
          //return 1;
        }
        if (client_play.call(srv_play))
        {
          std::cout<<srv_play.response.message<<"\n";
        }
        else
        {
          ROS_ERROR("Failed to call service play");
          //return 1;
        }
        ros::Duration(1.0).sleep();
      }

      void reachHomePosition()
      {
        //same values for joint names all the time so initializing them only once
        goal_ur5.trajectory.points[0].positions.clear();
        goal_ur5.trajectory.points[0].positions.resize(6);
        goal_ur5.trajectory.points[0].positions[0] = -0.032111946736471;
        goal_ur5.trajectory.points[0].positions[1] = -0.5465276877032679;
        goal_ur5.trajectory.points[0].positions[2] = -2.0321715513812464;
        goal_ur5.trajectory.points[0].positions[3] = -2.22761041322817;
        goal_ur5.trajectory.points[0].positions[4] = 1.5238990783691406;
        goal_ur5.trajectory.points[0].positions[5] = -0.03807098070253545;
        //movej([-0.888646427785055, -1.6399744192706507, -2.104286495839254, -1.0045259634601038, 1.4438568353652954, 2.1107845306396484], a=0.3, v=0.13)
        std::string cmd = "movej(["+to_string(goal_ur5.trajectory.points[0].positions[0])+","+to_string(goal_ur5.trajectory.points[0].positions[1])+","+to_string(goal_ur5.trajectory.points[0].positions[2])+","+to_string(goal_ur5.trajectory.points[0].positions[3])+","+to_string(goal_ur5.trajectory.points[0].positions[4])+","+to_string(goal_ur5.trajectory.points[0].positions[5])+"], a=0.3, v=0.2)";
        std_msgs::String command;
        command.data = cmd;
        pub_command.publish(command);
      }

      void pickUpObject()
      {
        //clear positions datas
        goal_ur5.trajectory.points[0].positions.resize(6);
        goal_ur5.trajectory.points[0].positions.clear();
        
        //reach pickup point
        //first approach
        goal_ur5.trajectory.points[0].positions[0] = -0.8898676077472132;
        goal_ur5.trajectory.points[0].positions[1] = -1.4975102583514612;
        goal_ur5.trajectory.points[0].positions[2] = -1.8582385222064417;
        goal_ur5.trajectory.points[0].positions[3] = -1.3935759703265589;
        goal_ur5.trajectory.points[0].positions[4] = 1.441737174987793;
        goal_ur5.trajectory.points[0].positions[5] = 2.1152422428131104;
        
        std::string cmd = "movej(["+to_string(goal_ur5.trajectory.points[0].positions[0])+","+to_string(goal_ur5.trajectory.points[0].positions[1])+","+to_string(goal_ur5.trajectory.points[0].positions[2])+","+to_string(goal_ur5.trajectory.points[0].positions[3])+","+to_string(goal_ur5.trajectory.points[0].positions[4])+","+to_string(goal_ur5.trajectory.points[0].positions[5])+"], a=0.3, v=0.25)";
        std_msgs::String command;
        command.data = cmd;
        pub_command.publish(command);
        ros::Duration(7.0).sleep();
        //clear positions datas
        goal_ur5.trajectory.points[0].positions.clear();

        //second approach
        goal_ur5.trajectory.points[0].positions[0] = -0.888646427785055;
        goal_ur5.trajectory.points[0].positions[1] = -1.6399744192706507;
        goal_ur5.trajectory.points[0].positions[2] = -2.104286495839254;
        goal_ur5.trajectory.points[0].positions[3] = -1.0045259634601038;
        goal_ur5.trajectory.points[0].positions[4] = 1.4438568353652954;
        goal_ur5.trajectory.points[0].positions[5] = 2.1107845306396484;
        
        cmd = "movej(["+to_string(goal_ur5.trajectory.points[0].positions[0])+","+to_string(goal_ur5.trajectory.points[0].positions[1])+","+to_string(goal_ur5.trajectory.points[0].positions[2])+","+to_string(goal_ur5.trajectory.points[0].positions[3])+","+to_string(goal_ur5.trajectory.points[0].positions[4])+","+to_string(goal_ur5.trajectory.points[0].positions[5])+"], a=0.3, v=0.25)";
        command.data = cmd;
        pub_command.publish(command);
        ros::Duration(3.0).sleep();
        //close gripper
        loadRosControl();
        ros::Duration(0.8).sleep();
        closeGripper();
        ros::Duration(1.0).sleep();
      }

      void placeObjectBorderOne()
      {
        //clear positions datas
        goal_ur5.trajectory.points[0].positions.resize(6);
        goal_ur5.trajectory.points[0].positions.clear();

        //go to border 1
        //first approach
        goal_ur5.trajectory.points[0].positions[0] = -0.6835039297686976;
        goal_ur5.trajectory.points[0].positions[1] = -2.15231162706484;
        goal_ur5.trajectory.points[0].positions[2] = -1.1330259482013147;
        goal_ur5.trajectory.points[0].positions[3] = -1.4790852705584925;
        goal_ur5.trajectory.points[0].positions[4] = 1.4701941013336182;
        goal_ur5.trajectory.points[0].positions[5] = 0.8814712166786194;
        
        std::string cmd = "movej(["+to_string(goal_ur5.trajectory.points[0].positions[0])+","+to_string(goal_ur5.trajectory.points[0].positions[1])+","+to_string(goal_ur5.trajectory.points[0].positions[2])+","+to_string(goal_ur5.trajectory.points[0].positions[3])+","+to_string(goal_ur5.trajectory.points[0].positions[4])+","+to_string(goal_ur5.trajectory.points[0].positions[5])+"], a=0.3, v=0.25)";
        std_msgs::String command;
        command.data = cmd;
        pub_command.publish(command);
        ros::Duration(9.0).sleep();

        //clear positions datas
        goal_ur5.trajectory.points[0].positions.clear();

        //second approach
        goal_ur5.trajectory.points[0].positions[0] = -0.6835039297686976;
        goal_ur5.trajectory.points[0].positions[1] = -2.15231162706484;
        goal_ur5.trajectory.points[0].positions[2] = -1.1330259482013147;
        goal_ur5.trajectory.points[0].positions[3] = -1.4790852705584925;
        goal_ur5.trajectory.points[0].positions[4] = 1.4701941013336182;
        goal_ur5.trajectory.points[0].positions[5] = 0.8814712166786194;
        
        cmd = "movej(["+to_string(goal_ur5.trajectory.points[0].positions[0])+","+to_string(goal_ur5.trajectory.points[0].positions[1])+","+to_string(goal_ur5.trajectory.points[0].positions[2])+","+to_string(goal_ur5.trajectory.points[0].positions[3])+","+to_string(goal_ur5.trajectory.points[0].positions[4])+","+to_string(goal_ur5.trajectory.points[0].positions[5])+"], a=0.3, v=0.25)";
        command.data = cmd;
        pub_command.publish(command);
        ros::Duration(4.0).sleep();
        //open gripper
        loadRosControl();
        openGripper();

      }

      void placeObjectBorderTwo()
      {
        //go to border 2
        //clear positions datas
        goal_ur5.trajectory.points[0].positions.resize(6);
        goal_ur5.trajectory.points[0].positions.clear();
        //first approach
        goal_ur5.trajectory.points[0].positions[0] = -0.3464320341693323;
        goal_ur5.trajectory.points[0].positions[1] = -1.7149184385882776;
        goal_ur5.trajectory.points[0].positions[2] = -1.665729824696676;
        goal_ur5.trajectory.points[0].positions[3] = -1.4092887083636683;
        goal_ur5.trajectory.points[0].positions[4] = 1.4922438859939575;
        goal_ur5.trajectory.points[0].positions[5] = 1.2192466259002686;
        
        std::string cmd = "movej(["+to_string(goal_ur5.trajectory.points[0].positions[0])+","+to_string(goal_ur5.trajectory.points[0].positions[1])+","+to_string(goal_ur5.trajectory.points[0].positions[2])+","+to_string(goal_ur5.trajectory.points[0].positions[3])+","+to_string(goal_ur5.trajectory.points[0].positions[4])+","+to_string(goal_ur5.trajectory.points[0].positions[5])+"], a=0.3, v=0.25)";
        std_msgs::String command;
        command.data = cmd;
        pub_command.publish(command);
        ros::Duration(9.0).sleep();

        //clear positions datas
        goal_ur5.trajectory.points[0].positions.clear();

        //second approach
        goal_ur5.trajectory.points[0].positions[0] = -0.3456781546222132;
        goal_ur5.trajectory.points[0].positions[1] = -1.8131697813617151;
        goal_ur5.trajectory.points[0].positions[2] = -1.8484719435321253;
        goal_ur5.trajectory.points[0].positions[3] = -1.1279271284686487;
        goal_ur5.trajectory.points[0].positions[4] = 1.493537425994873;
        goal_ur5.trajectory.points[0].positions[5] = 1.2157483100891113;
        
        cmd = "movej(["+to_string(goal_ur5.trajectory.points[0].positions[0])+","+to_string(goal_ur5.trajectory.points[0].positions[1])+","+to_string(goal_ur5.trajectory.points[0].positions[2])+","+to_string(goal_ur5.trajectory.points[0].positions[3])+","+to_string(goal_ur5.trajectory.points[0].positions[4])+","+to_string(goal_ur5.trajectory.points[0].positions[5])+"], a=0.3, v=0.25)";
        command.data = cmd;
        pub_command.publish(command);
        ros::Duration(4.0).sleep();
        //open gripper
        loadRosControl();
        ros::Duration(0.8).sleep();
        openGripper();
        ros::Duration(1.0).sleep();
      }

      void placeObjectBorderThree()
      {
        //go to border 3
        //clear positions datas
        goal_ur5.trajectory.points[0].positions.resize(6);
        goal_ur5.trajectory.points[0].positions.clear();
        //first approach
        goal_ur5.trajectory.points[0].positions[0] = 0.15866851806640625;
        goal_ur5.trajectory.points[0].positions[1] = -1.5024526754962366;
        goal_ur5.trajectory.points[0].positions[2] = -1.9495251814471644;
        goal_ur5.trajectory.points[0].positions[3] = -1.3590200583087366;
        goal_ur5.trajectory.points[0].positions[4] = 1.5376120805740356;
        goal_ur5.trajectory.points[0].positions[5] = 1.7226570844650269;
        
        std::string cmd = "movej(["+to_string(goal_ur5.trajectory.points[0].positions[0])+","+to_string(goal_ur5.trajectory.points[0].positions[1])+","+to_string(goal_ur5.trajectory.points[0].positions[2])+","+to_string(goal_ur5.trajectory.points[0].positions[3])+","+to_string(goal_ur5.trajectory.points[0].positions[4])+","+to_string(goal_ur5.trajectory.points[0].positions[5])+"], a=0.3, v=0.25)";
        std_msgs::String command;
        command.data = cmd;
        pub_command.publish(command);
        ros::Duration(7.0).sleep();

        //clear positions datas
        goal_ur5.trajectory.points[0].positions.clear();

        //second approach
        goal_ur5.trajectory.points[0].positions[0] = 0.159481942653656;
        goal_ur5.trajectory.points[0].positions[1] = -1.5931018034564417;
        goal_ur5.trajectory.points[0].positions[2] = -2.094579044972555;
        goal_ur5.trajectory.points[0].positions[3] = -1.1232979933368128;
        goal_ur5.trajectory.points[0].positions[4] = 1.5386539697647095;
        goal_ur5.trajectory.points[0].positions[5] = 1.7200089693069458;
        
        cmd = "movej(["+to_string(goal_ur5.trajectory.points[0].positions[0])+","+to_string(goal_ur5.trajectory.points[0].positions[1])+","+to_string(goal_ur5.trajectory.points[0].positions[2])+","+to_string(goal_ur5.trajectory.points[0].positions[3])+","+to_string(goal_ur5.trajectory.points[0].positions[4])+","+to_string(goal_ur5.trajectory.points[0].positions[5])+"], a=0.3, v=0.25)";
        command.data = cmd;
        pub_command.publish(command);
        ros::Duration(3.0).sleep();
        //open gripper
        loadRosControl();
        openGripper();
      }

      void placeObjectBorderFour()
      {
        //go to border 4
        //clear positions datas
        goal_ur5.trajectory.points[0].positions.resize(6);
        goal_ur5.trajectory.points[0].positions.clear();
        //first approach
        goal_ur5.trajectory.points[0].positions[0] = 0.6501914262771606;
        goal_ur5.trajectory.points[0].positions[1] = -1.5820687452899378;
        goal_ur5.trajectory.points[0].positions[2] = -1.7957366148578089;
        goal_ur5.trajectory.points[0].positions[3] = -1.430507008229391;
        goal_ur5.trajectory.points[0].positions[4] = 1.585481882095337;
        goal_ur5.trajectory.points[0].positions[5] = 2.212498188018799;
        
        std::string cmd = "movej(["+to_string(goal_ur5.trajectory.points[0].positions[0])+","+to_string(goal_ur5.trajectory.points[0].positions[1])+","+to_string(goal_ur5.trajectory.points[0].positions[2])+","+to_string(goal_ur5.trajectory.points[0].positions[3])+","+to_string(goal_ur5.trajectory.points[0].positions[4])+","+to_string(goal_ur5.trajectory.points[0].positions[5])+"], a=0.3, v=0.25)";
        std_msgs::String command;
        command.data = cmd;
        pub_command.publish(command);
        ros::Duration(7.0).sleep();

        //clear positions datas
        goal_ur5.trajectory.points[0].positions.clear();

        //second approach
        goal_ur5.trajectory.points[0].positions[0] = 0.651041567325592;
        goal_ur5.trajectory.points[0].positions[1] = -1.6847642103778284;
        goal_ur5.trajectory.points[0].positions[2] = -1.9885085264789026;
        goal_ur5.trajectory.points[0].positions[3] = -1.1350515524493616;
        goal_ur5.trajectory.points[0].positions[4] = 1.586906909942627;
        goal_ur5.trajectory.points[0].positions[5] = 2.209130048751831;
        
        cmd = "movej(["+to_string(goal_ur5.trajectory.points[0].positions[0])+","+to_string(goal_ur5.trajectory.points[0].positions[1])+","+to_string(goal_ur5.trajectory.points[0].positions[2])+","+to_string(goal_ur5.trajectory.points[0].positions[3])+","+to_string(goal_ur5.trajectory.points[0].positions[4])+","+to_string(goal_ur5.trajectory.points[0].positions[5])+"], a=0.3, v=0.25)";
        command.data = cmd;
        pub_command.publish(command);
        ros::Duration(3.0).sleep();
        //open gripper
        loadRosControl();
        openGripper();
      }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_case");
  ros::NodeHandle nh;
  UR5 robot(&nh);
  ros::Duration(2.5).sleep();
  //robot.reachHomePosition();
  //robot.loadRosControl();
  //while(!robot.getIncoming())
  //{
  //  ros::spinOnce();
  //}
  //robot.openGripper();
  //robot.reachHomePosition();
  //robot.pickUpObject(); 
  //robot.placeObjectBorderTwo();
  //robot.reachHomePosition();
  
  
  Client_layout client_layout("execution/projector_interface/integration/actions/set_layout_static_borders", true);
  client_layout.waitForServer();
  SetLayoutStaticBordersGoal layout_goal;
  layout_goal.book_adjacent = true;
  layout_goal.request_id = "09d-d09d9-fgd";
  layout_goal.safety_factor = 1.0;
  layout_goal.size_cols = 4;
  layout_goal.size_rows = 1;
  layout_goal.status_booked.r = 1.0;
  layout_goal.status_booked.g = 0.0;
  layout_goal.status_booked.b = 0.0;
  layout_goal.status_booked.a = 0.0;
  layout_goal.status_free.r = 0.0;
  layout_goal.status_free.g = 1.0;
  layout_goal.status_free.b = 0.0;
  layout_goal.status_free.a = 0.0;
  layout_goal.status_operator.r = 0.0;
  layout_goal.status_operator.g = 0.0;
  layout_goal.status_operator.b = 1.0;
  layout_goal.status_operator.a = 0.0;

  client_layout.sendGoal(layout_goal);
  client_layout.waitForResult(ros::Duration(5.0));
  if (client_layout.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("layout sent!");
  printf("Current State: %s\n", client_layout.getState().toString().c_str());

  ros::Duration(3.0).sleep();
  Client client("/execution/projector_interface/integration/actions/set_preset_ui_projection", true);
  client.waitForServer();
  Client_instruct client_inst("execution/projector_interface/integration/actions/set_instructions_projection", true);
  client_inst.waitForServer();
  SetPresetUIProjectionGoal msg_goal;
  //ros::NodeHandle nh;
  //ros::Publisher pub_ui = nh.advertise<SetPresetUIProjectionActionGoal>("/execution/projector_interface/integration/actions/set_preset_ui_projection/goal_robot",1);
  //SetPresetUIProjectionActionGoal msg_goal;
  VirtualButtonReference button_1;
  VirtualButtonReference button_2;
  VirtualButtonReference button_3;
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
  button_1.hidden = false;
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
  button_2.hidden = false;
  button_3.id = "30";
  button_3.zone = "table";
  button_3.name = "table";
  button_3.description = "button test";
  button_3.text = "TEST";
  button_3.button_color.r = 0.0;
  button_3.button_color.g = 1.0;
  button_3.button_color.b = 0.0;
  button_3.button_color.a = 0.0;
  button_3.text_color.r = 1.0;
  button_3.text_color.g = 1.0;
  button_3.text_color.b = 1.0;
  button_3.text_color.a = 1.0;
  button_3.center.position.x = 250.0;
  button_3.center.position.y = 100.0;
  button_3.radius = 70.0;
  button_3.hidden = false;
  msg_goal.request_id = "98097";
  msg_goal.zone = "table";
  msg_goal.resource_id = "01";
  msg_goal.name = "default ui";
  msg_goal.description = "default ui";
  msg_goal.hidden = false;
  msg_goal.virtual_button_references.clear();
  msg_goal.virtual_button_references.push_back(button_1);
  msg_goal.virtual_button_references.push_back(button_2);
  //msg_goal.virtual_button_references.push_back(button_3);


  client.sendGoal(msg_goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("interface sent!");
  printf("Current State: %s\n", client.getState().toString().c_str());

  ros::Duration(3.0).sleep();

  /*SetInstructionsProjectionGoal msg_inst;
  msg_inst.request_id = "286";
  msg_inst.zone = "table";
  msg_inst.target_location.position.x = 250.0;
  msg_inst.target_location.position.y = 500.0;
  msg_inst.title = "instructions";
  msg_inst.title_color.r = 0.0;
  msg_inst.title_color.g = 1.0;
  msg_inst.title_color.b = 0.0;
  msg_inst.title_color.a = 0.0;
  msg_inst.description = "press start to launch the next task";
  msg_inst.description_color.r = 0.0;
  msg_inst.description_color.g = 0.5;
  msg_inst.description_color.b = 1.0;
  msg_inst.description_color.a = 0.0;
  msg_inst.lifetime.fromNSec(0);
  client_inst.sendGoal(msg_inst);
  client_inst.waitForResult(ros::Duration(5.0));
  if (client_inst.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("instruction sent!");
  printf("Current State: %s\n", client_inst.getState().toString().c_str());*/

  Client_static_border client_border("/execution/projector_interface/integration/actions/set_safety_border_projection", true);
  client_border.waitForServer();
  SetSafetyBorderProjectionGoal msg;
  msg.request_id = "4";
  msg.zone = "shelf";
  msg.position_row = 0;
  msg.position_col = 3;
  geometry_msgs::PolygonStamped pl;
  msg.border.polygon.points.clear();
  msg.border.header.frame_id = "base";
  msg.border.header.stamp = ros::Time::now();
  geometry_msgs::Point32 p;
  p.x = 0.5; //1.73
  p.y = 0.35; //-0.08
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  p.x = 0.7; //0.5
  p.y = 0.15; 
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  msg.border_topic = "";
  msg.border_color.r = 0.0;
  msg.border_color.g = 1.0;
  msg.border_color.b = 0.0;
  msg.border_color.a = 0.0;
  msg.is_filled = false;
  msg.thickness = 1;
  msg.lifetime.fromNSec(0);
  msg.track_violations = true;
  client_border.sendGoal(msg);
  client_border.waitForResult(ros::Duration(5.0));
  if (client_border.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("static border sent!");
  printf("Current State: %s\n", client_border.getState().toString().c_str());

  ros::Duration(1.0).sleep();  
  msg.request_id = "3";
  msg.zone = "shelf";
  msg.position_row = 0;
  msg.position_col = 2;
  //geometry_msgs::PolygonStamped pl;
  msg.border.polygon.points.clear();
  msg.border.header.frame_id = "base";
  msg.border.header.stamp = ros::Time::now();
  //geometry_msgs::Point32 p;
  p.x = 0.5; //1.73
  p.y = 0.1; //-0.08
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  p.x = 0.7; //0.5
  p.y = -0.1; 
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  msg.border_topic = "";
  msg.border_color.r = 0.0;
  msg.border_color.g = 1.0;
  msg.border_color.b = 0.0;
  msg.border_color.a = 0.0;
  msg.is_filled = false;
  msg.thickness = 1;
  msg.lifetime.fromNSec(0);
  msg.track_violations = true;
  client_border.sendGoal(msg);
  client_border.waitForResult(ros::Duration(5.0));
  if (client_border.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("static border sent!");
  printf("Current State: %s\n", client_border.getState().toString().c_str());
  ros::Duration(1.0).sleep();
  
  msg.request_id = "2";
  msg.zone = "shelf";
  msg.position_row = 0;
  msg.position_col = 1;
  //geometry_msgs::PolygonStamped pl;
  msg.border.polygon.points.clear();
  msg.border.header.frame_id = "base";
  msg.border.header.stamp = ros::Time::now();
  //geometry_msgs::Point32 p;
  p.x = 0.5; //1.73
  p.y = -0.15; //-0.08
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  p.x = 0.7; //0.5
  p.y = -0.35; 
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  msg.border_topic = "";
  msg.border_color.r = 0.0;
  msg.border_color.g = 1.0;
  msg.border_color.b = 0.0;
  msg.border_color.a = 0.0;
  msg.is_filled = false;
  msg.thickness = 1;
  msg.lifetime.fromNSec(0);
  msg.track_violations = true;
  client_border.sendGoal(msg);
  client_border.waitForResult(ros::Duration(5.0));
  if (client_border.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("static border sent!");
  printf("Current State: %s\n", client_border.getState().toString().c_str());
  
  ros::Duration(1.0).sleep();
  msg.request_id = "1";
  msg.zone = "shelf";
  msg.position_row = 0;
  msg.position_col = 0;
  //geometry_msgs::PolygonStamped pl;
  msg.border.polygon.points.clear();
  msg.border.header.frame_id = "base";
  msg.border.header.stamp = ros::Time::now();
  //geometry_msgs::Point32 p;
  p.x = 0.5; //1.73
  p.y = -0.4; //-0.08
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  p.x = 0.7; //0.5
  p.y = -0.6; 
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  msg.border_topic = "";
  msg.border_color.r = 0.0;
  msg.border_color.g = 1.0;
  msg.border_color.b = 0.0;
  msg.border_color.a = 0.0;
  msg.is_filled = false;
  msg.thickness = 1;
  msg.lifetime.fromNSec(0);
  msg.track_violations = true;
  client_border.sendGoal(msg);
  client_border.waitForResult(ros::Duration(5.0));
  if (client_border.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("static border sent!");
  printf("Current State: %s\n", client_border.getState().toString().c_str());

  ros::Duration(5.0).sleep();

  Client_book_robot client_book_rob("execution/projector_interface/integration/actions/set_book_robot_static_border", true);
  client_book_rob.waitForServer();
  BookRobotStaticBorderGoal robot_booking;
  robot_booking.request_id = "29796";
  robot_booking.id = "2";
  client_book_rob.sendGoal(robot_booking);
  client_book_rob.waitForResult(ros::Duration(5.0));
  if (client_book_rob.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("border booked !");
  printf("Current State: %s\n", client_book_rob.getState().toString().c_str());

  ros::Duration(5.0).sleep();

  robot.pickUpObject();
  robot.placeObjectBorderTwo();
  robot.reachHomePosition();

  ros::Duration(5.0).sleep();

  Client_release_robot client_rel_rob("execution/projector_interface/integration/actions/set_release_robot_static_border", true);
  client_rel_rob.waitForServer();
  ReleaseRobotStaticBorderGoal robot_rel;
  robot_rel.request_id = "26968";
  robot_rel.id = "2";
  robot_rel.status = -1;
  client_rel_rob.sendGoal(robot_rel);
  client_rel_rob.waitForResult(ros::Duration(5.0));
  if (client_rel_rob.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("border released !");
  printf("Current State: %s\n", client_rel_rob.getState().toString().c_str());
  
  ros::Duration(5.0).sleep();

  Client_book_operator client_book_op("execution/projector_interface/integration/actions/set_book_operator_static_border", true);
  client_book_op.waitForServer();
  BookOperatorStaticBorderGoal op_booking;
  op_booking.request_id = "97345464";
  op_booking.id = "2";
  client_book_op.sendGoal(op_booking);
  client_book_op.waitForResult(ros::Duration(5.0));
  if (client_book_op.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("border operator booked !");
  printf("Current State: %s\n", client_book_op.getState().toString().c_str());

  ros::Duration(5.0).sleep();

  Client_release_operator client_rel_op("execution/projector_interface/integration/actions/set_release_operator_static_border", true);
  client_rel_op.waitForServer();
  ReleaseOperatorStaticBorderGoal op_rel;
  op_rel.request_id = "987964";
  op_rel.id = "2";
  op_rel.status = 0;
  client_rel_op.sendGoal(op_rel);
  client_rel_op.waitForResult(ros::Duration(5.0));
  if (client_rel_op.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("border released !");
  printf("Current State: %s\n", client_rel_op.getState().toString().c_str());
  
  ros::spin();



  
  return 0;
}