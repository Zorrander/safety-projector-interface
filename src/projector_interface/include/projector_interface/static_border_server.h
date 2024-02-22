#ifndef StaticBorderServer_H
#define StaticBorderServer_H


#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>

#include <integration/SetLayoutStaticBordersAction.h>

#include "border/DynamicBorder.h"
#include "border/StaticBorder.h"
#include "border/StaticBorderManager.h"

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

using namespace integration;


class StaticBorderServer
{
   protected:
      
      // NodeHandle instance must be created before this line. Otherwise strange error occurs.
      actionlib::SimpleActionServer<SetLayoutStaticBordersAction> as_border_manager;
      std::string action_name_border_manager_;
      bool activate_static_border_manager;
      bool activate_dynamic_border;
      bool add_static_border;
      bool book_operator_border;
      bool release_operator_border;
      bool book_robot_border;
      bool release_robot_border;
      //std::vector<> vec_borders;
      boost::shared_ptr<ros::AsyncSpinner> g_spinner;
      // create messages that are used to published feedback/result
      SetLayoutStaticBordersActionFeedback feedback_layout;
      SetLayoutStaticBordersResult result_layout;
      

      //attributes to monitor situation
      //monitor active buttons and publish them
      std::vector<std::string> displayed_request_ids;
      std::vector<std::string> displayed_ids_borders;
      std::vector<std::string> border_robot_booked;
      std::vector<std::string> border_operator_booked;
      std::vector<BorderStatus> release_border_robot;
      std::vector<BorderStatus> release_border_operator;

      std::vector<StaticBorder> l_borders;



   public:
      StaticBorderServer(ros::NodeHandle *nh_, std::string name_bd_manager);
      //send feedback
      void sendFeedbackSetLayout(std::string id_goal);
      //send result
      void sendResultSetLayout(std::string id_goal);
      //create a border layout. This will launch a border manager with a thread.
      //The tread keeps the border manager running and ready to create or book borders.
      void executeSetLayout(const SetLayoutStaticBordersGoalConstPtr &goal);

};
#endif