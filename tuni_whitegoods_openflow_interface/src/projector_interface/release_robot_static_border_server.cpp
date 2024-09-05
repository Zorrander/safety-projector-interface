#include "projector_interface/release_robot_static_border_server.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/ReleaseRobotStaticBorderAction.h>



      ReleaseRobotStaticBorderServer::ReleaseRobotStaticBorderServer(ros::NodeHandle *nh_, std::string name_release_robot, std::shared_ptr<ProjectorInterfaceController> projector_interface_controller) :
      // Bind the callback to the action server. False is for thread spinning
      as_release_robot(*nh_, name_release_robot, boost::bind(&ReleaseRobotStaticBorderServer::executeReleaseRobot, this, _1), false),
      action_name_release_robot(name_release_robot),
      controller(projector_interface_controller)
      {
         as_release_robot.start();
         std::cout<<"ReleaseRobotStaticBorderServer running\n";
      }

      //release robot border
      void ReleaseRobotStaticBorderServer::executeReleaseRobot(const ReleaseRobotStaticBorderGoalConstPtr &goal)
      {
         controller->robot_release_border(goal->id, goal->status);
         bool success = true;
         sendFeedbackReleaseRobot(goal->request_id);
         if (as_release_robot.isPreemptRequested() || !ros::ok())
         {
            ROS_INFO("%s: Preempted", action_name_release_robot.c_str());
            // set the action state to preempted
            as_release_robot.setPreempted();
            success = false;
         }
         if(success)
         {
            ROS_INFO("%s: Succeeded", action_name_release_robot.c_str());
            // set the action state to succeeded
            sendResultReleaseRobot(goal->request_id);
         }
      }
      //send feedback
      void ReleaseRobotStaticBorderServer::sendFeedbackReleaseRobot(std::string req_id)
      {
         feedback_release_robot_.feedback.displayed_request_ids.clear();
         feedback_release_robot_.feedback.displayed_request_ids.push_back(req_id);
         as_release_robot.publishFeedback(feedback_release_robot_.feedback);
      }
      //send result
      void ReleaseRobotStaticBorderServer::sendResultReleaseRobot(std::string req_id)
      {
         result_release_robot_.displayed_request_ids.clear();
         result_release_robot_.displayed_request_ids.push_back(req_id);
         as_release_robot.setSucceeded(result_release_robot_);
      }