#ifndef UserInterfaceServer_H
#define UserInterfaceServer_H

#include <actionlib/server/simple_action_server.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <memory>
#include <string>

#include "integration/ProjectorUI.h"
#include "integration/SetPresetUIProjectionAction.h"
#include "integration/VirtualButtonReference.h"
#include "tuni_whitegoods_controller/projector_interface_controller.h"

using namespace integration;

class UserInterfaceServer {
 protected:
  actionlib::SimpleActionServer<SetPresetUIProjectionAction> as_preset_ui;
  std::string action_name_preset_ui;

  // create messages that are used to published feedback/result
  SetPresetUIProjectionActionFeedback feedback_preset_ui;
  SetPresetUIProjectionResult result_preset_ui;

  std::shared_ptr<ProjectorInterfaceController> controller;

 public:
  UserInterfaceServer(ros::NodeHandle *nh_, std::string name_pre,
                      std::shared_ptr<ProjectorInterfaceController>
                          projector_interface_controller);
  // create a smart interface
  void executePresetUI(const SetPresetUIProjectionGoalConstPtr &goal);
  // create the smart interface by sending the infos to the projector python
  // code through ROS publisher
  void createPresetUIMsg(const SetPresetUIProjectionGoalConstPtr &goal);
  // send feedback
  void sendFeedBackPresetUI();
  // send result
  void sendResultPresetUI();
};
#endif