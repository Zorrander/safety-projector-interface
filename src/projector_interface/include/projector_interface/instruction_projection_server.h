#ifndef InstructionProjectionServer_H
#define InstructionProjectionServer_H

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <integration/SetInstructionsProjectionAction.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <integration/VirtualButtonReference.h>
#include <integration/ProjectorUI.h>
#include <unity_msgs/Instructions.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

using namespace integration;


class InstructionProjectionServer
{
   protected:
      
      actionlib::SimpleActionServer<SetInstructionsProjectionAction> as_instruct;
      std::string action_name_instruct;

      //std::vector<> vec_borders;

      // create messages that are used to published feedback/result
      SetInstructionsProjectionActionFeedback feedback_instruct_;
      SetInstructionsProjectionResult result_instruct_;
      

      //attributes to monitor situation
      //monitor active buttons and publish them
      std::vector<std::string> displayed_request_ids;
      ros::Publisher pub_instruction;



   public:
      InstructionProjectionServer(ros::NodeHandle *nh_, std::string name_in);
      //send instruction to be written on the interface
      void executeInstruction(const SetInstructionsProjectionGoalConstPtr &goal);
      //send feedback
      void sendFeedBackInstruction();
      //send result
      void sendResultInstruction();

};
#endif