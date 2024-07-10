#include "projector_interface/static_border_server.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/SetLayoutStaticBordersAction.h>

      StaticBorderServer::StaticBorderServer(ros::NodeHandle *nh, std::string name_bd_manager) :
      // Bind the callback to the action server. False is for thread spinning
      //nh_border_(*n),
      as_border_manager(*nh, name_bd_manager, boost::bind(&StaticBorderServer::executeSetLayout, this, _1), false),
      action_name_border_manager_(name_bd_manager),
      nh_(nh)
      {
         //Start prerequisites
         displayed_request_ids.clear();
         border_robot_booked.clear();
         border_operator_booked.clear();
         release_border_robot.clear();
         release_border_operator.clear();
         // Start the action server
         as_border_manager.start();
         add_static_border = false;
         ros::param::get("book_robot_static_border_server", book_robot_static_border_server_name);
         ros::param::get("release_robot_static_border_server", release_robot_static_border_server_name);
         ros::param::get("book_operator_static_border_server", book_operator_static_border_server_name);
         ros::param::get("release_operator_static_border_server", release_operator_static_border_server_name);
         ros::param::get("safety_border_server", safety_border_server_name);

      }

      //create a border layout. This will launch a border manager with a thread.
      //The tread keeps the border manager running and ready to create or book borders.
      void StaticBorderServer::executeSetLayout(const SetLayoutStaticBordersGoalConstPtr &goal)
      {
         bool success = true;

         sbm_ = std::make_shared<StaticBorderManager>(nh_, goal->size_rows, goal->size_cols, goal->safety_factor, goal->book_adjacent, goal->status_booked, goal->status_free, goal->status_operator);

         ROS_INFO("BookRobotStaticBorderServer starting");
         book_robot_static_border_server = new BookRobotStaticBorderServer(nh_, book_robot_static_border_server_name, sbm_);
         ROS_INFO("ReleaseRobotStaticBorderServer starting");
         release_robot_static_border_server = new ReleaseRobotStaticBorderServer(nh_, release_robot_static_border_server_name, sbm_);
         ROS_INFO("BookOperatorStaticBorderServer starting");
         book_operator_static_border_server = new  BookOperatorStaticBorderServer(nh_, book_operator_static_border_server_name, sbm_);
         ROS_INFO("ReleaseOperatorStaticBorderServer starting");
         release_operator_static_border_server = new ReleaseOperatorStaticBorderServer(nh_, release_operator_static_border_server_name, sbm_);
         ROS_INFO("SafetyBorderServer starting");
         safety_border_server = new SafetyBorderServer(nh_, safety_border_server_name, sbm_);

         displayed_ids_borders.push_back(goal->request_id);
         sendFeedbackSetLayout(goal->request_id);
         if (as_border_manager.isPreemptRequested() || !ros::ok() || !success)
         {
            ROS_INFO("%s: Preempted", action_name_border_manager_.c_str());
            // set the action state to preempted
            as_border_manager.setPreempted();
            success = false;
         }
         if(success)
         {
            ROS_INFO("%s: Succeeded", action_name_border_manager_.c_str());
            // set the action state to succeeded
            sendResultSetLayout(goal->request_id);
         }
      }
      //send feedback
      void StaticBorderServer::sendFeedbackSetLayout(std::string id_goal)
      {
         feedback_layout.feedback.displayed_request_ids.clear();
         feedback_layout.feedback.displayed_request_ids.push_back(id_goal);
         as_border_manager.publishFeedback(feedback_layout.feedback);
      }
      //send result
      void StaticBorderServer::sendResultSetLayout(std::string id_goal)
      {
         result_layout.displayed_request_ids.clear();
         result_layout.displayed_request_ids.push_back(id_goal);
         as_border_manager.setSucceeded(result_layout);
      }

