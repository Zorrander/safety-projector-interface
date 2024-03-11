#ifndef UserInterfaceServer_H
#define UserInterfaceServer_H


#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/SetPresetUIProjectionAction.h>

#include <integration/VirtualButtonReference.h>
#include <integration/ProjectorUI.h>
#include <unity_msgs/Instructions.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

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
      UserInterfaceServer(ros::NodeHandle *nh_, std::string name_pre);
      //create a smart interface
      void executePresetUI(const SetPresetUIProjectionGoalConstPtr &goal);
      // create the smart interface by sending the infos to the projector python code through ROS publisher
      void createPresetUIMsg(const SetPresetUIProjectionGoalConstPtr &goal);
      //send feedback
      void sendFeedBackPresetUI();
      //send result
      void sendResultPresetUI();
 
};
#endif