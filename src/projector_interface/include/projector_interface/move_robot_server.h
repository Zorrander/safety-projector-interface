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


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Trajectory_action;

class MoveRobotServer
{
   protected:
      actionlib::SimpleActionServer<MoveJointsAction> as_move_joints;
      
      Trajectory_action *trajectory_ur5;

      std::string action_name_move_joints;
      // create messages that are used to published feedback/result
      MoveJointsActionFeedback feedback_mj;
      MoveJointsActionResult result_mj;
      ros::Publisher pub_cmd_robot;



   public:
      //move the joints of the robot to some positions
      void executeMoveJoints(const MoveJointsGoalConstPtr &goal);
      //send feedback
      void sendFeedbackMoveJoints(ActionRequest ar);
      //send result
      void sendResultMoveJoints(ActionRequest ar);
};


