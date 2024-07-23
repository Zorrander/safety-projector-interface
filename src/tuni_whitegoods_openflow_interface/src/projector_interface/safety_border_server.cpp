#include "projector_interface/safety_border_server.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/SetSafetyBorderProjectionAction.h>

#include "border/StaticBorder.h"
#include "border/StaticBorderManager.h"


      SafetyBorderServer::SafetyBorderServer(ros::NodeHandle *nh_, std::string name_border, std::shared_ptr<StaticBorderManager> sbm) :
      // Bind the callback to the action server. False is for thread spinning
      as_border(*nh_, name_border, boost::bind(&SafetyBorderServer::executeSafetyBorder, this, _1), false),
      action_name_border_(name_border),
      sbm(sbm)
      {
         //Start prerequisites
         displayed_request_ids.clear();
         as_border.start();
         std::cout<<"SafetyBorderServer running\n";
      }

      void SafetyBorderServer::executeSafetyBorder(const SetSafetyBorderProjectionGoalConstPtr &goal)
      {
         bool success = true;
         std::cout<<"executeSafetyBorder\n";
         if(goal->border.polygon.points.size() > 1)
         {
            //static border
            std::cout<<"adding static border...\n";

            // Dynamically allocate StaticBorder object using std::make_shared
            std::shared_ptr<StaticBorder> sb = std::make_shared<StaticBorder>(goal->request_id, goal->zone, goal->position_row, goal->position_col, goal->border, goal->border_topic, goal->border_color, goal->is_filled, goal->thickness, goal->lifetime, goal->track_violations);

            // Pass the shared_ptr to the addBorder method
            sbm->addBorder(sb);

            sbm->publishBorder();
            add_static_border = true;
         }
         displayed_ids_borders.push_back(goal->request_id);
         sendFeedBackBorder();
         if (as_border.isPreemptRequested() || !ros::ok() || !success)
         {
            ROS_INFO("%s: Preempted", action_name_border_.c_str());
            // set the action state to preempted
            as_border.setPreempted();
            success = false;
         }
         if(success)
         {
            ROS_INFO("%s: Succeeded", action_name_border_.c_str());
            // set the action state to succeeded
            sendResultBorder();
         }
      }
      
      //send feedback
      void SafetyBorderServer::sendFeedBackBorder()
      {
         feedback_border_.feedback.displayed_request_ids.clear();
         for(std::string i : displayed_ids_borders)
         {
            feedback_border_.feedback.displayed_request_ids.push_back(i);
         }
         as_border.publishFeedback(feedback_border_.feedback);
      }
      //send result
      void SafetyBorderServer::sendResultBorder()
      {
         result_border_.displayed_request_ids.clear();
         for(std::string i : displayed_ids_borders)
         {
            result_border_.displayed_request_ids.push_back(i);
         }
         as_border.setSucceeded(result_border_);
      }

