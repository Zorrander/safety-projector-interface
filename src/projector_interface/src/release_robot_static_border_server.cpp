
#include "projector_interface/release_robot_static_border_server.h"


using namespace integration;


      ReleaseRobotStaticBorderServer(ros::NodeHandle *nh_, std::string name_release_robot,) :
      // Bind the callback to the action server. False is for thread spinning
      as_release_robot(*nh_, name_release_robot, boost::bind(&ReleaseRobotStaticBorderServer::executeReleaseRobot, this, _1), false),
      action_name_release_robot(name_release_robot),
      {
         release_border_robot.clear();
         as_release_robot.start();
      }

      //release robot border
      void executeReleaseRobot(const ReleaseRobotStaticBorderGoalConstPtr &goal)
      {
         BorderStatus bs;
         bs.id_border = goal->id;
         bs.status = goal->status;
         release_border_robot.push_back(bs);
         release_robot_border = true;
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
      void sendFeedbackReleaseRobot(std::string req_id)
      {
         feedback_release_robot_.feedback.displayed_request_ids.clear();
         feedback_release_robot_.feedback.displayed_request_ids.push_back(req_id);
         as_release_robot.publishFeedback(feedback_release_robot_.feedback);
      }
      //send result
      void sendResultReleaseRobot(std::string req_id)
      {
         result_release_robot_.displayed_request_ids.clear();
         result_release_robot_.displayed_request_ids.push_back(req_id);
         as_release_robot.setSucceeded(result_release_robot_);
      }