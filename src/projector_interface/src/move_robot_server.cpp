/*
The class is the openflow server that will receives all the actions coming from openflow
*/
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/SetVirtualButtonsProjectionAction.h>
#include <integration/SetVirtualButtonChangeColorAction.h>
#include <integration/SetSafetyBorderProjectionAction.h>
#include <integration/SetPresetUIProjectionAction.h>
#include <integration/SetLayoutStaticBordersAction.h>
#include <integration/UnsetProjectionAction.h>
#include <integration/SetInstructionsProjectionAction.h>
#include <integration/BookRobotStaticBorderAction.h>
#include <integration/ReleaseRobotStaticBorderAction.h>
#include <integration/BookOperatorStaticBorderAction.h>
#include <integration/ReleaseOperatorStaticBorderAction.h>
#include <integration/MoveJointsAction.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <integration/VirtualButtonReference.h>
#include <integration/ProjectorUI.h>
#include <unity_msgs/Instructions.h>
#include "border/DynamicBorder.hpp"
#include "border/StaticBorder.hpp"
#include "border/StaticBorderManager.hpp"
#include <thread>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

using namespace integration;


      MoveRobotServer(ros::NodeHandle *nh_, std::string name_mj) :
      // Bind the callback to the action server. False is for thread spinning
      as_move_joints(*nh_, name_mj, boost::bind(&MoveRobotServer::executeMoveJoints, this, _1), false),

      action_name_move_joints(name_mj)
      {
         pub_cmd_robot = nh_->advertise<std_msgs::String>("/ur_hardware_interface/script_command",1);

         as_move_joints.start();
 
      }

      // Destructor
      ~ProjectorInterface(void)
      {
         delete trajectory_ur5;
      }

      //move the joints of the robot to some positions
      void executeMoveJoints(const MoveJointsGoalConstPtr &goal)
      {
         control_msgs::FollowJointTrajectoryGoal goal_ur5;
         goal_ur5.trajectory = goal->trajectory;
         std::string cmd = "movej(["+to_string(goal->trajectory.points[0].positions[0])+","+to_string(goal->trajectory.points[0].positions[1])+","+to_string(goal->trajectory.points[0].positions[2])+","+to_string(goal->trajectory.points[0].positions[3])+","+to_string(goal->trajectory.points[0].positions[4])+","+to_string(goal->trajectory.points[0].positions[5])+"], a=0.3, v=0.13)";
         std_msgs::String command;
         command.data = cmd;
         pub_cmd_robot.publish(command);
         ros::Duration(7.0).sleep();
         bool success = true;
         sendFeedbackMoveJoints(goal->action_request);
         if (as_move_joints.isPreemptRequested() || !ros::ok())
         {
            ROS_INFO("%s: Preempted", action_name_move_joints.c_str());
            // set the action state to preempted
            as_move_joints.setPreempted();
            success = false;
         }
         if(success)
         {
            ROS_INFO("%s: Succeeded", action_name_move_joints.c_str());
            // set the action state to succeeded
            sendResultMoveJoints(goal->action_request);
         }
      }
      //send feedback
      void sendFeedbackMoveJoints(ActionRequest ar)
      {
         MoveJointsFeedback af;
         af.action_feedback.action_id = ar.action_id;
         af.action_feedback.millis_passed = ar.expected_duration_millis;
         as_move_joints.publishFeedback(af);
      }
      //send result
      void sendResultMoveJoints(ActionRequest ar)
      {
         MoveJointsResult mjr;
         mjr.action_result.action_id = ar.action_id;
         mjr.action_result.millis_passed = ar.expected_duration_millis;
         as_move_joints.setSucceeded(mjr);
      }



