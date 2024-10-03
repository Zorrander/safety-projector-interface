#ifndef SafetyBorderServer_H
#define SafetyBorderServer_H

#include <actionlib/server/simple_action_server.h>
#include <integration/SetSafetyBorderProjectionAction.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <memory>
#include <string>
#include <thread>

#include "tuni_whitegoods_controller/projector_interface_controller.h"

using namespace integration;

class SafetyBorderServer {
 public:
  SafetyBorderServer(ros::NodeHandle *nh_, std::string name_border,
                     std::shared_ptr<ProjectorInterfaceController>
                         projector_interface_controller);

  // create a static or dynamic border. For a static border, it just pass the
  // information to the already running thread.
  // For the dynamic border, it starts a dynamic border thread.
  void executeSafetyBorder(const SetSafetyBorderProjectionGoalConstPtr &goal);
  // send feedback
  void sendFeedBackBorder();
  // send result
  void sendResultBorder();

 protected:
  actionlib::SimpleActionServer<SetSafetyBorderProjectionAction> as_border;
  std::string action_name_border_;

  // create messages that are used to published feedback/result
  SetSafetyBorderProjectionActionFeedback feedback_border_;
  SetSafetyBorderProjectionResult result_border_;

  std::shared_ptr<ProjectorInterfaceController> controller;
};
#endif