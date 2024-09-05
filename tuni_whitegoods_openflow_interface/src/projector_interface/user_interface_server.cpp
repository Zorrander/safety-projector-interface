#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/SetPresetUIProjectionAction.h>

#include "projector_interface/user_interface_server.h"


      UserInterfaceServer::UserInterfaceServer(ros::NodeHandle *nh_, std::string name_pre, std::shared_ptr<ProjectorInterfaceController> projector_interface_controller) :
      as_preset_ui(*nh_, name_pre, boost::bind(&UserInterfaceServer::executePresetUI, this, _1), false),
      action_name_preset_ui(name_pre),
      controller(projector_interface_controller)
      {
         as_preset_ui.start();
         std::cout<<"UserInterfaceServer running \n";
      }

      //create a smart interface
      void UserInterfaceServer::executePresetUI(const SetPresetUIProjectionGoalConstPtr &goal)
      {
         for (auto button: goal->virtual_button_references)
         {
            controller->addButton(button.id, button.zone, button.name, button.text, 
                                 button.button_color, button.text_color, 
                                 button.center, button.radius);
         }
         bool success = true;

         sendFeedBackPresetUI();
         if (as_preset_ui.isPreemptRequested() || !ros::ok())
         {
            ROS_INFO("%s: Preempted", action_name_preset_ui.c_str());
            // set the action state to preempted
            as_preset_ui.setPreempted();
            success = false;
         }
         if(success)
         {
            //result_.displayed_request_ids.clear();
            ROS_INFO("%s: Succeeded", action_name_preset_ui.c_str());
            // set the action state to succeeded
            sendResultPresetUI();
         }
      }

      //send feedback
      void UserInterfaceServer::sendFeedBackPresetUI()
      {
         feedback_preset_ui.feedback.displayed_request_ids.clear();
         as_preset_ui.publishFeedback(feedback_preset_ui.feedback);
      }
      //send result
      void UserInterfaceServer::sendResultPresetUI()
      {
         result_preset_ui.displayed_request_ids.clear();
         as_preset_ui.setSucceeded(result_preset_ui);
      }
