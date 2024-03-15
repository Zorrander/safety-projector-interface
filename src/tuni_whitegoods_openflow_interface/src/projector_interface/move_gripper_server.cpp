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
       }

      //move the joints of the robot to some positions
      void MoveGripperServer::executeMoveGripper(const ControlOpenCloseToolGoalConstPtr &goal)
      {
         ROS_INFO("MoveRobotServer::executeMoveGripper");
         bool success = true;
         sendFeedbackMoveGripper(goal->action_request);
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



