#ifndef ReleaseRobotStaticBorderServer_H
#define ReleaseRobotStaticBorderServer_H

#include "border/DynamicBorder.h"
#include "border/StaticBorder.h"

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>

#include <integration/ReleaseRobotStaticBorderAction.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

using namespace integration;

class StaticBorderManager;

class ReleaseRobotStaticBorderServer
{
   protected:
      
      actionlib::SimpleActionServer<ReleaseRobotStaticBorderAction> as_release_robot;

      std::string action_name_release_robot;

      bool release_robot_border;

      std::vector<StaticBorder> l_borders;
      // create messages that are used to published feedback/result
      ReleaseRobotStaticBorderActionFeedback feedback_release_robot_;
      ReleaseRobotStaticBorderResult result_release_robot_;

      std::shared_ptr<StaticBorderManager> sbm;

      //attributes to monitor situation
      //monitor active buttons and publish them
      std::vector<std::string> displayed_request_ids;
      std::vector<BorderStatus> release_border_robot;


   public:
      ReleaseRobotStaticBorderServer(ros::NodeHandle *nh_, std::string name_release_robot, std::shared_ptr<StaticBorderManager> sbm);
      //release robot border
      void executeReleaseRobot(const ReleaseRobotStaticBorderGoalConstPtr &goal);
      //send feedback
      void sendFeedbackReleaseRobot(std::string req_id);
      //send result
      void sendResultReleaseRobot(std::string req_id);

};
#endif