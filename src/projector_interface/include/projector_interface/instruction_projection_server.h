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


class InstructionProjectionServer
{
   protected:
      actionlib::SimpleActionServer<SetInstructionsProjectionAction> as_instruct;
      std::string action_name_instruct;

      //std::vector<> vec_borders;

      // create messages that are used to published feedback/result
      SetInstructionsProjectionActionFeedback feedback_instruct_;
      SetInstructionsProjectionResult result_instruct_;
      

      //attributes to monitor situation
      //monitor active buttons and publish them
      std::vector<std::string> displayed_request_ids;
      ros::Publisher pub_instruction;



   public:
      //send instruction to be written on the interface
      void executeInstruction(const SetInstructionsProjectionGoalConstPtr &goal);
      //send feedback
      void sendFeedBackInstruction();
      //send result
      void sendResultInstruction();

};