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

class UserInterfaceServer
{
   protected:
      actionlib::SimpleActionServer<SetPresetUIProjectionAction> as_preset_ui;
      std::string action_name_preset_ui;

      // create messages that are used to published feedback/result
      SetPresetUIProjectionActionFeedback feedback_preset_ui;
      SetPresetUIProjectionResult result_preset_ui;

      //attributes to monitor situation
      //monitor active buttons and publish them
      std::vector<std::string> displayed_request_ids;
      ros::Publisher pub_preset_ui;



   public:
      //create a smart interface
      void executePresetUI(const SetPresetUIProjectionGoalConstPtr &goal);
      // create the smart interface by sending the infos to the projector python code through ROS publisher
      void createPresetUIMsg(const SetPresetUIProjectionGoalConstPtr &goal);
      //send feedback
      void sendFeedBackPresetUI();
      //send result
      void sendResultPresetUI();
 
};
