#ifndef UnsetProjectionServer_H
#define UnsetProjectionServer_H

#include <actionlib/server/simple_action_server.h>
#include <memory>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>

#include <integration/UnsetProjectionAction.h>

#include <integration/ProjectorUI.h>
#include <integration/VirtualButtonReference.h>

#include "tuni_whitegoods_controller/projector_interface_controller.h"

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

using namespace integration;

class UnsetProjectionServer {
protected:
  actionlib::SimpleActionServer<UnsetProjectionAction> as_unset;
  std::string action_name_unset;

  UnsetProjectionActionFeedback feedback_unset_;
  UnsetProjectionResult result_unset_;
  bool activate_dynamic_border;
  bool activate_static_border_manager;

  std::shared_ptr<ProjectorInterfaceController> controller;

public:
  UnsetProjectionServer(ros::NodeHandle *nh_, std::string name_un,
                        std::shared_ptr<ProjectorInterfaceController>
                            projector_interface_controller);
  // unset the smart interface
  void executeUnsetUI(const UnsetProjectionGoalConstPtr &goal);
  // send feedback
  void sendFeedBackUnset();
  // send result
  void sendResultUnset();
};

#endif