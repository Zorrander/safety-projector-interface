#ifndef ButtonProjectionServer_H
#define ButtonProjectionServer_H

#include <actionlib/server/simple_action_server.h>
#include <integration/SetVirtualButtonsProjectionAction.h>
#include <ros/ros.h>

#include <memory>
#include <string>

#include "tuni_whitegoods_controller/projector_interface_controller.h"

using namespace integration;

class ButtonProjectionServer {
 protected:
  actionlib::SimpleActionServer<SetVirtualButtonsProjectionAction> as_;

  std::string action_name_button_;
  std::vector<std::string> displayed_request_ids;

  // create messages that are used to published feedback/result
  SetVirtualButtonsProjectionActionFeedback feedback_button_;
  SetVirtualButtonsProjectionActionResult result_;

  std::shared_ptr<ProjectorInterfaceController> controller;

 public:
  ButtonProjectionServer(ros::NodeHandle* nh_, std::string name_vb,
                         std::shared_ptr<ProjectorInterfaceController>
                             projector_interface_controller);
  // create a virtual button
  void executeVirtualButtonsGoal(
      const SetVirtualButtonsProjectionGoalConstPtr& goal);
  // send feedback
  void sendFeedBackButton(std::string id_goal);
  // send result
  void sendResultButton(std::string id_goal);
};
#endif
