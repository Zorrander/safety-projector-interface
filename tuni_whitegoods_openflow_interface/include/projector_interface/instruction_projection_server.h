#ifndef InstructionProjectionServer_H
#define InstructionProjectionServer_H

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <memory>
#include <string>

#include "integration/SetInstructionsProjectionAction.h"
#include "tuni_whitegoods_controller/projector_interface_controller.h"

using namespace integration;

class InstructionProjectionServer {
 protected:
  actionlib::SimpleActionServer<SetInstructionsProjectionAction> as_instruct;
  std::string action_name_instruct;

  // std::vector<> vec_borders;

  // create messages that are used to published feedback/result
  SetInstructionsProjectionActionFeedback feedback_instruct_;
  SetInstructionsProjectionResult result_instruct_;

  std::shared_ptr<ProjectorInterfaceController> controller;

  // attributes to monitor situation
  // monitor active buttons and publish them
  std::vector<std::string> displayed_request_ids;
  ros::Publisher pub_instruction;

 public:
  InstructionProjectionServer(ros::NodeHandle *nh_, std::string name_in,
                              std::shared_ptr<ProjectorInterfaceController>
                                  projector_interface_controller);
  // send instruction to be written on the interface
  void executeInstruction(const SetInstructionsProjectionGoalConstPtr &goal);
  // send feedback
  void sendFeedBackInstruction();
  // send result
  void sendResultInstruction();
};
#endif