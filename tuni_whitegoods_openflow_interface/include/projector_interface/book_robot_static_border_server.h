#ifndef BookRobotStaticBorderServer_H
#define BookRobotStaticBorderServer_H

#include <memory>
#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <integration/BookRobotStaticBorderAction.h>

#include "tuni_whitegoods_controller/projector_interface_controller.h"

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

using namespace integration;

class BookRobotStaticBorderServer {
protected:
  actionlib::SimpleActionServer<BookRobotStaticBorderAction> as_book_robot;
  std::string action_name_book_robot;
  bool book_robot_border;
  BookRobotStaticBorderActionFeedback feedback_book_robot_;
  BookRobotStaticBorderResult result_book_robot_;

  std::shared_ptr<ProjectorInterfaceController> controller;

  std::vector<std::string> border_robot_booked;
  std::vector<std::string> displayed_request_ids;

public:
  BookRobotStaticBorderServer(ros::NodeHandle *nh_, std::string name_book_robot,
                              std::shared_ptr<ProjectorInterfaceController>
                                  projector_interface_controller);
  
  void executeBookRobot(const BookRobotStaticBorderGoalConstPtr& goal);
  
  void sendFeedbackBookRobot(std::string req_id);

  void sendResultBookRobot(std::string req_id);
};
#endif