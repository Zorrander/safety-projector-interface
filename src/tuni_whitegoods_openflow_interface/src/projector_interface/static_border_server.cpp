#include "projector_interface/static_border_server.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/SetLayoutStaticBordersAction.h>

      StaticBorderServer::StaticBorderServer(ros::NodeHandle *nh, std::string name_bd_manager, std::shared_ptr<ProjectorInterfaceController> projector_interface_controller) :
      // Bind the callback to the action server. False is for thread spinning
      as_border_manager(*nh, name_bd_manager, boost::bind(&StaticBorderServer::executeSetLayout, this, _1), false),
      action_name_border_manager_(name_bd_manager),
      controller(projector_interface_controller),
      nh_(nh)
      {
         // Start the action server
         as_border_manager.start();
      }

      void StaticBorderServer::executeSetLayout(const SetLayoutStaticBordersGoalConstPtr &goal)
      {
         bool success = true;

         controller->createBorderLayout(goal->size_rows, goal->size_cols, goal->safety_factor, goal->book_adjacent, goal->status_booked, goal->status_free, goal->status_operator);

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

