#ifndef ReleaseOperatorStaticBorderServer_H
#define ReleaseOperatorStaticBorderServer_H


#include "border/StaticBorder.h"

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>

#include <integration/ReleaseOperatorStaticBorderAction.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

using namespace integration;

class StaticBorderManager;

class ReleaseOperatorStaticBorderServer
{
   protected:
      
      actionlib::SimpleActionServer<ReleaseOperatorStaticBorderAction> as_release_operator;
      std::string action_name_release_operator;
      bool release_operator_border;
      //std::vector<> vec_borders;
      std::vector<StaticBorder> l_borders;
      // create messages that are used to published feedback/result
      ReleaseOperatorStaticBorderActionFeedback feedback_release_operator_;
      ReleaseOperatorStaticBorderResult result_release_operator;

      std::shared_ptr<StaticBorderManager> sbm;
      //attributes to monitor situation
      //monitor active buttons and publish them
      std::vector<std::string> displayed_request_ids;
      std::vector<BorderStatus> release_border_operator;


   public:
      ReleaseOperatorStaticBorderServer(ros::NodeHandle *nh_, std::string name_release_operator, std::shared_ptr<StaticBorderManager> sbm);
      //release operator border
      void executeReleaseOperator(const ReleaseOperatorStaticBorderGoalConstPtr &goal);
      //send feedback
      void sendFeedbackReleaseOperator(std::string req_id);
      //send result
      void sendResultReleaseOperator(std::string req_id);
 
};
#endif