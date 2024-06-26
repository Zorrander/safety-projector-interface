#ifndef SafetyBorderServer_H
#define SafetyBorderServer_H



#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>

#include <integration/SetSafetyBorderProjectionAction.h>

#include "border/DynamicBorder.h"
#include "border/StaticBorder.h"

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>
#include <thread>

using namespace integration;

class StaticBorderManager;

class SafetyBorderServer
{
   public:
      SafetyBorderServer(ros::NodeHandle *nh_, std::string name_border, std::shared_ptr<StaticBorderManager> sbm);

      //create a static or dynamic border. For a static border, it just pass the information to the already running thread.
      // For the dynamic border, it starts a dynamic border thread.
      void executeSafetyBorder(const SetSafetyBorderProjectionGoalConstPtr &goal);
      //send feedback
      void sendFeedBackBorder();
      //send result
      void sendResultBorder();
      
   protected:
      actionlib::SimpleActionServer<SetSafetyBorderProjectionAction> as_border;
      std::string action_name_border_;
      boost::shared_ptr<ros::AsyncSpinner> g_spinner;
      bool activate_dynamic_border;
      bool activate_static_border_manager;
      std::unique_ptr<std::thread> th_b;
      bool add_static_border;
      ros::CallbackQueue queue;
      std::shared_ptr<StaticBorderManager> sbm;
      // create messages that are used to published feedback/result
      SetSafetyBorderProjectionActionFeedback feedback_border_;
      SetSafetyBorderProjectionResult result_border_;
      std::vector<StaticBorder> l_borders;
      //attributes to monitor situation
      //monitor active buttons and publish them
      std::vector<std::string> displayed_request_ids;
      std::vector<std::string> displayed_ids_borders;

      void threadCreateDynamicBorder(ros::NodeHandle nh_, std::string r_id, std::string z, std::string b_topic, std_msgs::ColorRGBA b_color, bool filling, int thic, ros::Duration life, bool track);

};
#endif