#include "projector_interface/instruction_projection_server.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/SetInstructionsProjectionAction.h>



      InstructionProjectionServer::InstructionProjectionServer(ros::NodeHandle *nh_, std::string name_in) :
      // Bind the callback to the action server. False is for thread spinning
      as_instruct(*nh_, name_in, boost::bind(&InstructionProjectionServer::executeInstruction, this, _1), false),

      action_name_instruct(name_in)
      {

         displayed_request_ids.clear();
         as_instruct.start();
      }

      //send instruction to be written on the interface
      void InstructionProjectionServer::executeInstruction(const SetInstructionsProjectionGoalConstPtr &goal)
      {
         std::cout<<"got instruction !\n";
         ros::Rate r(1);
         bool success = true;
         unity_msgs::Instructions msg;
         displayed_request_ids.push_back(goal->request_id);
         msg.request_id = goal->request_id;
         msg.zone = goal->zone;
         msg.target_location = goal->target_location;
         msg.title = goal->title;
         msg.title_color = goal->title_color;
         msg.description = goal->description;
         msg.description_color = goal->description_color;
         msg.lifetime = goal->lifetime;
         pub_instruction.publish(msg);
         sendFeedBackInstruction();
         if (as_instruct.isPreemptRequested() || !ros::ok())
         {
            ROS_INFO("%s: Preempted", action_name_instruct.c_str());
            // set the action state to preempted
            as_instruct.setPreempted();
            success = false;
         }
         if(success)
         {
            ROS_INFO("%s: Succeeded", action_name_instruct.c_str());
            // set the action state to succeeded
            sendResultInstruction();
         }
      }
      //send feedback
      void InstructionProjectionServer::sendFeedBackInstruction()
      {
         feedback_instruct_.feedback.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            feedback_instruct_.feedback.displayed_request_ids.push_back(i);
         }
         as_instruct.publishFeedback(feedback_instruct_.feedback);
      }
      //send result
      void InstructionProjectionServer::sendResultInstruction()
      {
         result_instruct_.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            result_instruct_.displayed_request_ids.push_back(i);
         }
         as_instruct.setSucceeded(result_instruct_);
      }

