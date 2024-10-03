#ifndef ButtonColorServer_H
#define ButtonColorServer_H

#include <actionlib/server/simple_action_server.h>
#include <integration/SetVirtualButtonChangeColorAction.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <memory>
#include <string>

#include "tuni_whitegoods_controller/projector_interface_controller.h"

using namespace integration;

class ButtonColorServer {
 protected:
  actionlib::SimpleActionServer<SetVirtualButtonChangeColorAction> as_change;
  std::string action_name_color_;

  SetVirtualButtonChangeColorActionFeedback feedback_color_;
  SetVirtualButtonChangeColorResult result_color_;

  std::shared_ptr<ProjectorInterfaceController> controller;

 public:
  ButtonColorServer(ros::NodeHandle* nh_, std::string name_cc,
                    std::shared_ptr<ProjectorInterfaceController>
                        projector_interface_controller);

  // change button color
  void executeChangeButtonColor(
      const SetVirtualButtonChangeColorGoalConstPtr& goal);
  // senf feedback
  void sendFeedBackChangeButton(std::string id_goal);
  // send result
  void sendResultChangeButton(std::string id_goal);
};
#endif