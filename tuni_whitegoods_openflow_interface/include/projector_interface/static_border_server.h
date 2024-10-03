#ifndef StaticBorderServer_H
#define StaticBorderServer_H

#include <actionlib/server/simple_action_server.h>
#include <integration/SetLayoutStaticBordersAction.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <memory>
#include <string>

#include "tuni_whitegoods_controller/projector_interface_controller.h"

using namespace integration;

class StaticBorderServer {
 protected:
  ros::NodeHandle *nh_;
  // NodeHandle instance must be created before this line. Otherwise strange
  // error occurs.
  actionlib::SimpleActionServer<SetLayoutStaticBordersAction> as_border_manager;
  std::string action_name_border_manager_;

  // create messages that are used to published feedback/result
  SetLayoutStaticBordersActionFeedback feedback_layout;
  SetLayoutStaticBordersResult result_layout;
  std::shared_ptr<ProjectorInterfaceController> controller;

 public:
  StaticBorderServer(ros::NodeHandle *nh_, std::string name_bd_manager,
                     std::shared_ptr<ProjectorInterfaceController>
                         projector_interface_controller);
  // send feedback
  void sendFeedbackSetLayout(std::string id_goal);
  // send result
  void sendResultSetLayout(std::string id_goal);
  // create a border layout. This will launch a border manager with a thread.
  // The tread keeps the border manager running and ready to create or book
  // borders.
  void executeSetLayout(const SetLayoutStaticBordersGoalConstPtr &goal);
};
#endif