/*
The class is the openflow server that will receives all the actions coming from openflow
*/
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/SetVirtualButtonsProjectionAction.h>
#include <integration/SetVirtualButtonChangeColorAction.h>
#include <integration/SetSafetyBorderProjectionAction.h>
#include <integration/SetPresetUIProjectionAction.h>
#include <integration/SetLayoutStaticBordersAction.h>
#include <integration/UnsetProjectionAction.h>
#include <integration/SetInstructionsProjectionAction.h>
#include <integration/BookRobotStaticBorderAction.h>
#include <integration/ReleaseRobotStaticBorderAction.h>
#include <integration/BookOperatorStaticBorderAction.h>
#include <integration/ReleaseOperatorStaticBorderAction.h>
#include <integration/MoveJointsAction.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <integration/VirtualButtonReference.h>
#include <integration/ProjectorUI.h>
#include <unity_msgs/Instructions.h>
#include "border/DynamicBorder.hpp"
#include "border/StaticBorder.hpp"
#include "border/StaticBorderManager.hpp"
#include <thread>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

using namespace integration;

struct BorderStatus{
   std::string id_border;
   int status;
};

class ReleaseOperatorStaticBorderServer
{
   protected:
      actionlib::SimpleActionServer<ReleaseOperatorStaticBorderAction> as_release_operator;
      std::string action_name_release_operator;
      bool release_operator_border;
      //std::vector<> vec_borders;

      // create messages that are used to published feedback/result
      ReleaseOperatorStaticBorderActionFeedback feedback_release_operator_;
      ReleaseOperatorStaticBorderResult result_release_operator;

      //attributes to monitor situation
      //monitor active buttons and publish them
      std::vector<std::string> displayed_request_ids;
      std::vector<BorderStatus> release_border_operator;


   public:
      //release operator border
      void executeReleaseOperator(const ReleaseOperatorStaticBorderGoalConstPtr &goal);
      //send feedback
      void sendFeedbackReleaseOperator(std::string req_id);
      //send result
      void sendResultReleaseOperator(std::string req_id);
 
};