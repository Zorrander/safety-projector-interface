#ifndef BookOperatorStaticBorderServer_H
#define BookOperatorStaticBorderServer_H

#include <actionlib/server/simple_action_server.h>
#include <integration/BookOperatorStaticBorderAction.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <memory>
#include <string>

#include "tuni_whitegoods_controller/projector_interface_controller.h"

using namespace integration;

class BookOperatorStaticBorderServer {
 protected:
  // NodeHandle instance must be created before this line. Otherwise strange
  // error occurs.
  actionlib::SimpleActionServer<BookOperatorStaticBorderAction>
      as_book_operator;

  std::string action_name_book_operator;
  bool book_operator_border;

  std::shared_ptr<ProjectorInterfaceController> controller;

  // create messages that are used to published feedback/result
  BookOperatorStaticBorderActionFeedback feedback_book_operator_;
  BookOperatorStaticBorderResult result_book_operator_;

  // attributes to monitor situation
  // monitor active buttons and publish them
  std::vector<std::string> displayed_request_ids;
  std::vector<std::string> border_operator_booked;
  std::vector<std::string> release_border_operator;

 public:
  BookOperatorStaticBorderServer(ros::NodeHandle* nh_,
                                 std::string name_book_operator,
                                 std::shared_ptr<ProjectorInterfaceController>
                                     projector_interface_controller);

  void executeBookOperator(const BookOperatorStaticBorderGoalConstPtr& goal);

  void sendFeedbackBookOperator(std::string req_id);

  void sendResultBookOperator(std::string req_id);
};
#endif