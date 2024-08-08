#include "projector_interface/release_operator_static_border_server.h"

#include "projector_interface_controller.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/ReleaseOperatorStaticBorderAction.h>


      ReleaseOperatorStaticBorderServer::ReleaseOperatorStaticBorderServer(ros::NodeHandle *nh_, std::string name_release_operator, std::shared_ptr<ProjectorInterfaceController> projector_interface_controller) :
      // Bind the callback to the action server. False is for thread spinning
      as_release_operator(*nh_, name_release_operator, boost::bind(&ReleaseOperatorStaticBorderServer::executeReleaseOperator, this, _1), false),
      action_name_release_operator(name_release_operator),
      controller(projector_interface_controller)
      {
         //Start prerequisites
         displayed_request_ids.clear();
         as_release_operator.start();
         std::cout<<"ReleaseOperatorStaticBorderServer running\n";
      }


      //release operator border
      void ReleaseOperatorStaticBorderServer::executeReleaseOperator(const ReleaseOperatorStaticBorderGoalConstPtr &goal)
      {

         bool success = true;
         sendFeedbackReleaseOperator(goal->request_id);
         if (as_release_operator.isPreemptRequested() || !ros::ok())
         {
            ROS_INFO("%s: Preempted", action_name_release_operator.c_str());
            // set the action state to preempted
            as_release_operator.setPreempted();
            success = false;
         }
         if(success)
         {
            ROS_INFO("%s: Succeeded", action_name_release_operator.c_str());
            // set the action state to succeeded
            sendResultReleaseOperator(goal->request_id);
         }
      }
      //send feedback
      void ReleaseOperatorStaticBorderServer::sendFeedbackReleaseOperator(std::string req_id)
      {
         feedback_release_operator_.feedback.displayed_request_ids.clear();
         feedback_release_operator_.feedback.displayed_request_ids.push_back(req_id);
         as_release_operator.publishFeedback(feedback_release_operator_.feedback);
      }
      //send result
      void ReleaseOperatorStaticBorderServer::sendResultReleaseOperator(std::string req_id)
      {
         result_release_operator.displayed_request_ids.clear();
         result_release_operator.displayed_request_ids.push_back(req_id);
         as_release_operator.setSucceeded(result_release_operator);
      }
 