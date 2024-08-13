#include "projector_interface/unset_projection_server.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/UnsetProjectionAction.h>


      UnsetProjectionServer::UnsetProjectionServer(ros::NodeHandle *nh_, std::string name_un, std::shared_ptr<ProjectorInterfaceController> projector_interface_controller) :
      as_unset(*nh_, name_un, boost::bind(&UnsetProjectionServer::executeUnsetUI, this, _1), false),
      action_name_unset(name_un),
      controller(projector_interface_controller)
      {
         as_unset.start();
         std::cout<<"UnsetProjectionServer running \n";
      }

      //unset the smart interface
      void UnsetProjectionServer::executeUnsetUI(const UnsetProjectionGoalConstPtr &goal)
      {
         bool success = true;
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
         as_unset.publishFeedback(feedback_unset_.feedback);
      }
      //send result
      void UnsetProjectionServer::sendResultUnset()
      {
         result_unset_.displayed_request_ids.clear();
         as_unset.setSucceeded(result_unset_);
      }

