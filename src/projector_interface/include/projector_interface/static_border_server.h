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

class StaticBorderServer
{
   protected:
      // NodeHandle instance must be created before this line. Otherwise strange error occurs.
      actionlib::SimpleActionServer<SetLayoutStaticBordersAction> as_border_manager;
      std::string action_name_border_manager_;
      bool activate_static_border_manager;

      //std::vector<> vec_borders;

      // create messages that are used to published feedback/result
      SetLayoutStaticBordersActionFeedback feedback_layout;
      SetLayoutStaticBordersResult result_layout;
      

      //attributes to monitor situation
      //monitor active buttons and publish them
      std::vector<std::string> displayed_request_ids;
      std::vector<std::string> displayed_ids_borders;
      std::vector<std::string> border_robot_booked;
      std::vector<std::string> border_operator_booked;
      std::vector<BorderStatus> release_border_robot;
      std::vector<BorderStatus> release_border_operator;

      std::vector<StaticBorder> l_borders;



   public:
      //send feedback
      void sendFeedbackSetLayout(std::string id_goal);
      //send result
      void sendResultSetLayout(std::string id_goal);
      //create a border layout. This will launch a border manager with a thread.
      //The tread keeps the border manager running and ready to create or book borders.
      void executeSetLayout(const SetLayoutStaticBordersGoalConstPtr &goal);
      //create a static or dynamic border. For a static border, it just pass the information to the already running thread.
      // For the dynamic border, it starts a dynamic border thread.
      void executeSafetyBorder(const SetSafetyBorderProjectionGoalConstPtr &goal);
      
};
