#include "projector_server/release_operator_static_border_server.h"


using namespace integration;

      ReleaseOperatorStaticBorderServer(ros::NodeHandle *nh_, std::string name_release_operator) :
      // Bind the callback to the action server. False is for thread spinning
      as_release_operator(*nh_, name_release_operator, boost::bind(&ReleaseOperatorStaticBorderServer::executeReleaseOperator, this, _1), false),
      action_name_release_operator(name_release_operator)
      {
         //Start prerequisites
         displayed_request_ids.clear();
         release_border_operator.clear();
         as_release_operator.start();

      }


      //release operator border
      void executeReleaseOperator(const ReleaseOperatorStaticBorderGoalConstPtr &goal)
      {
         BorderStatus bs;
         bs.id_border = goal->id;
         bs.status = goal->status;
         release_border_operator.push_back(bs);
         release_operator_border = true;
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
      void sendFeedbackReleaseOperator(std::string req_id)
      {
         feedback_release_operator_.feedback.displayed_request_ids.clear();
         feedback_release_operator_.feedback.displayed_request_ids.push_back(req_id);
         as_release_operator.publishFeedback(feedback_release_operator_.feedback);
      }
      //send result
      void sendResultReleaseOperator(std::string req_id)
      {
         result_release_operator.displayed_request_ids.clear();
         result_release_operator.displayed_request_ids.push_back(req_id);
         as_release_operator.setSucceeded(result_release_operator);
      }
 