#include "projector_interface/static_border_server.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/SetLayoutStaticBordersAction.h>

      StaticBorderServer::StaticBorderServer(ros::NodeHandle *nh_, std::string name_bd_manager) :
      // Bind the callback to the action server. False is for thread spinning
      //nh_border_(*n),
      as_border_manager(*nh_, name_bd_manager, boost::bind(&StaticBorderServer::executeSetLayout, this, _1), false),
      action_name_border_manager_(name_bd_manager)
      {
         //Start prerequisites
         displayed_request_ids.clear();
         border_robot_booked.clear();
         border_operator_booked.clear();
         release_border_robot.clear();
         release_border_operator.clear();
         // Start the action server
         as_border_manager.start();
         activate_static_border_manager = false;
         add_static_border = false;
      }

      //create a border layout. This will launch a border manager with a thread.
      //The tread keeps the border manager running and ready to create or book borders.
      void StaticBorderServer::executeSetLayout(const SetLayoutStaticBordersGoalConstPtr &goal)
      {
         bool success = true;
         if(!activate_static_border_manager)
         {
            activate_static_border_manager = true;
         }
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

