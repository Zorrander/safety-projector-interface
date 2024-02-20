#include "projector_interface/user_interface_server.h"


using namespace integration;

      UserInterfaceServer(ros::NodeHandle *nh_, std::string name_pre) :
      as_preset_ui(*nh_, name_pre, boost::bind(&UserInterfaceServer::executePresetUI, this, _1), false),
      action_name_preset_ui(name_pre)
      {
         //Start prerequisites
         displayed_request_ids.clear();
         pub_preset_ui = nh_->advertise<ProjectorUI>("/interfaceUI/openflow/new_interface",1);
         as_preset_ui.start();
 
      }

      //create a smart interface
      void executePresetUI(const SetPresetUIProjectionGoalConstPtr &goal)
      {
         ros::Rate r(1);
         bool success = true;
         displayed_request_ids.push_back(goal->request_id);
         //send preset_ui
         createPresetUIMsg(goal);
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
      // create the smart interface by sending the infos to the projector python code through ROS publisher
      void createPresetUIMsg(const SetPresetUIProjectionGoalConstPtr &goal)
      {
         ProjectorUI msg;
         msg.resource_id = goal->resource_id;
         msg.zone = goal->zone;
         msg.name = goal->name;
         msg.description = goal->description;
         //msg.hidden = goal->hidden;
         msg.virtual_button_references.clear();
         for(VirtualButtonReference i : goal->virtual_button_references)
         {
            msg.virtual_button_references.push_back(i);
         }
         pub_preset_ui.publish(msg);
      }
      //send feedback
      void sendFeedBackPresetUI()
      {
         feedback_preset_ui.feedback.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            feedback_preset_ui.feedback.displayed_request_ids.push_back(i);
         }
         as_preset_ui.publishFeedback(feedback_preset_ui.feedback);
      }
      //send result
      void sendResultPresetUI()
      {
         result_preset_ui.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            result_preset_ui.displayed_request_ids.push_back(i);
         }
         as_preset_ui.setSucceeded(result_preset_ui);
      }
 
};
