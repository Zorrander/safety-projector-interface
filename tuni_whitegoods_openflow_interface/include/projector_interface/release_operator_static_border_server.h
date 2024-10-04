#ifndef ReleaseOperatorStaticBorderServer_H
#define ReleaseOperatorStaticBorderServer_H

#include <actionlib/server/simple_action_server.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <memory>
#include <string>

#include "integration/ReleaseOperatorStaticBorderAction.h"
#include "tuni_whitegoods_controller/projector_interface_controller.h"

using namespace integration;

class ReleaseOperatorStaticBorderServer {
 protected:
  actionlib::SimpleActionServer<ReleaseOperatorStaticBorderAction>
      as_release_operator;
  std::string action_name_release_operator;
  bool release_operator_border;
  // std::vector<> vec_borders;
  // create messages that are used to published feedback/result
  ReleaseOperatorStaticBorderActionFeedback feedback_release_operator_;
  ReleaseOperatorStaticBorderResult result_release_operator;

  std::shared_ptr<ProjectorInterfaceController> controller;

 public:
  ReleaseOperatorStaticBorderServer(
      ros::NodeHandle *nh_, std::string name_release_operator,
      std::shared_ptr<ProjectorInterfaceController>
          projector_interface_controller);
  // release operator border
  void executeReleaseOperator(
      const ReleaseOperatorStaticBorderGoalConstPtr &goal);
  // send feedback
  void sendFeedbackReleaseOperator(std::string req_id);
  // send result
  void sendResultReleaseOperator(std::string req_id);
};
#endif