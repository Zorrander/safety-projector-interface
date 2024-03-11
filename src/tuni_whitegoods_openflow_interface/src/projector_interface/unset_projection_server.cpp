#include "projector_interface/unset_projection_server.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/UnsetProjectionAction.h>


      UnsetProjectionServer::UnsetProjectionServer(ros::NodeHandle *nh_, std::string name_un) :
      as_unset(*nh_, name_un, boost::bind(&UnsetProjectionServer::executeUnsetUI, this, _1), false),
      action_name_unset(name_un)
      {
         //Start prerequisites
         displayed_request_ids.clear();
         pub_unset = nh_->advertise<std_msgs::Bool>("/interfaceUI/openflow/unset_projection",1);
         as_unset.start();
         std::cout<<"UnsetProjectionServer running \n";
      }

      //unset the smart interface
      void UnsetProjectionServer::executeUnsetUI(const UnsetProjectionGoalConstPtr &goal)
      {
         ros::Rate r(1);
         bool success = true;
         displayed_request_ids.push_back(goal->request_id);
         //unset border
         activate_dynamic_border = false;
         activate_static_border_manager = false;
         //unset projection
         std_msgs::Bool msg;
         msg.data = true;
         pub_unset.publish(msg);
         sendFeedBackUnset();
         if (as_unset.isPreemptRequested() || !ros::ok())
         {
            ROS_INFO("%s: Preempted", action_name_unset.c_str());
            // set the action state to preempted
            as_unset.setPreempted();
            success = false;
         }
         if(success)
         {
            ROS_INFO("%s: Succeeded", action_name_unset.c_str());
            // set the action state to succeeded
            sendResultUnset();
         }
      }
      //send feedback
      void UnsetProjectionServer::sendFeedBackUnset()
      {
         feedback_unset_.feedback.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            feedback_unset_.feedback.displayed_request_ids.push_back(i);
         }
         as_unset.publishFeedback(feedback_unset_.feedback);
      }
      //send result
      void UnsetProjectionServer::sendResultUnset()
      {
         result_unset_.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            result_unset_.displayed_request_ids.push_back(i);
         }
         as_unset.setSucceeded(result_unset_);
      }

