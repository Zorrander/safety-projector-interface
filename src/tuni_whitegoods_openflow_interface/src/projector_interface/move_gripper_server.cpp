#include "projector_interface/move_gripper_server.h"

#include <ros/ros.h>
#include <integration/MoveJointsAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>



      MoveGripperServer::MoveGripperServer(ros::NodeHandle *nh_, std::string name_mg) :
      // Bind the callback to the action server. False is for thread spinning
      as_move_gripper(*nh_, name_mg, boost::bind(&MoveGripperServer::executeMoveGripper, this, _1), false),
      action_name_move_gripper(name_mg)
      {
         pub_cmd_robot = nh_->advertise<std_msgs::String>("/ur_hardware_interface/script_command",1);
         as_move_gripper.start();
         client_load = nh_->serviceClient<ur_dashboard_msgs::Load>("/ur_hardware_interface/dashboard/load_program");
         client_play = nh_->serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/play");
       }

      //move the joints of the robot to some positions
      void MoveGripperServer::executeMoveGripper(const ControlOpenCloseToolGoalConstPtr &goal)
      {
         bool success = true;
         /*
         ROS_INFO("MoveRobotServer::executeMoveGripper");
         srv_req.request.filename = "ros_control.urp";
         success = run_dashboard_script(srv_req);
         ros::Duration(1.0).sleep();
         if (success){
            if (goal->tool_action.compare("close") == 0){
               srv_req.request.filename = "cl.urp";
               success = run_dashboard_script(srv_req); 
            } else if (goal->tool_action.compare("open") == 0){
               srv_req.request.filename = "open.urp";
               success = run_dashboard_script(srv_req);
            }
            sendFeedbackMoveGripper(goal->action_request);           
         }
         */

         if (as_move_gripper.isPreemptRequested() || !ros::ok())
         {
            ROS_INFO("%s: Preempted", action_name_move_gripper.c_str());
            // set the action state to preempted
            as_move_gripper.setPreempted();
            success = false;
         }

         if(success)
         {
            ROS_INFO("%s: Succeeded", action_name_move_gripper.c_str());
            // set the action state to succeeded
            sendResultMoveGripper(goal->action_request);
         }
      }

      bool MoveGripperServer::run_dashboard_script(ur_dashboard_msgs::Load srv_req)
      {
        bool success = true;

        if (client_load.call(srv_req))
        {
          std::cout<<srv_req.response.answer<<"\n";
        }
        else
        {
          ROS_ERROR("Failed to call service load");
          success = false;
          //return 1;
        }

        if (client_play.call(srv_play))
        {
          std::cout<<srv_play.response.message<<"\n";
        }
        else
        {
          ROS_ERROR("Failed to call service play");
          success = false;
          //return 1;
        }
        ros::Duration(1.0).sleep();
        return success;
      }

      //send feedback
      void MoveGripperServer::sendFeedbackMoveGripper(ActionRequest ar)
      {
         MoveJointsFeedback af;
         feedback_mgf.action_feedback.action_id = ar.action_id;
         feedback_mgf.action_feedback.millis_passed = ar.expected_duration_millis;
         as_move_gripper.publishFeedback(feedback_mgf);
      }
      //send result
      void MoveGripperServer::sendResultMoveGripper(ActionRequest ar)
      {
         result_mgr.action_result.action_id = ar.action_id;
         result_mgr.action_result.millis_passed = ar.expected_duration_millis;
         result_mgr.action_result.success = true;
         as_move_gripper.setSucceeded(result_mgr);
      }

