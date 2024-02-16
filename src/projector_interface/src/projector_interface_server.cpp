#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/SetVirtualButtonsProjectionAction.h>
#include <integration/SetVirtualButtonChangeColorAction.h>
#include <integration/SetSafetyBorderProjectionAction.h>
#include <integration/SetPresetUIProjectionAction.h>
#include <integration/UnsetProjectionAction.h>
#include <integration/SetInstructionsProjectionAction.h>
#include <integration/VirtualButtonReference.h>
#include <integration/ProjectorUI.h>
#include <unity_msgs/Instructions.h>
#include "border/DynamicBorder.hpp"
#include "border/StaticBorder.hpp"
#include "border/StaticBorderManager.hpp"
#include <thread>
#include <std_msgs/Bool.h>

using namespace integration;

class ProjectorInterface
{
   protected:
      boost::shared_ptr<ros::AsyncSpinner> g_spinner;
      ros::CallbackQueue queue;
      //ros::NodeHandle nh_;
      ros::NodeHandle nh_border_;
      boost::shared_ptr<ros::AsyncSpinner> static_border_spinner;
      ros::CallbackQueue static_border_queue;
      //ros::NodeHandle nh_;
      ros::NodeHandle nh_static_border_;
      // NodeHandle instance must be created before this line. Otherwise strange error occurs.
      actionlib::SimpleActionServer<SetVirtualButtonsProjectionAction> as_;
      actionlib::SimpleActionServer<SetVirtualButtonChangeColorAction> as_change;
      actionlib::SimpleActionServer<SetSafetyBorderProjectionAction> as_border;
      actionlib::SimpleActionServer<SetPresetUIProjectionAction> as_preset_ui;
      actionlib::SimpleActionServer<UnsetProjectionAction> as_unset;
      actionlib::SimpleActionServer<SetInstructionsProjectionAction> as_instruct;
      std::string action_name_button_;
      std::string action_name_color_;
      std::string action_name_border_;
      std::string action_name_preset_ui;
      std::string action_name_unset;
      std::string action_name_instruct;
      bool activate_dynamic_border;
      bool activate_static_border;
      bool add_static_border;
      //std::vector<> vec_borders;

      // create messages that are used to published feedback/result
      SetVirtualButtonsProjectionActionFeedback feedback_button_;
      SetVirtualButtonsProjectionActionResult result_;
      SetVirtualButtonChangeColorActionFeedback feedback_color_;
      SetVirtualButtonChangeColorResult result_color_;
      SetSafetyBorderProjectionActionFeedback feedback_border_;
      SetSafetyBorderProjectionResult result_border_;
      SetPresetUIProjectionActionFeedback feedback_preset_ui;
      SetPresetUIProjectionResult result_preset_ui;
      UnsetProjectionActionFeedback feedback_unset_;
      UnsetProjectionResult result_unset_;
      SetInstructionsProjectionActionFeedback feedback_instruct_;
      SetInstructionsProjectionResult result_instruct_;
      

      //attributes to monitor situation
      //monitor active buttons and publish them
      std::vector<std::string> displayed_request_ids;
      std::vector<std::string> displayed_ids_borders;
      ros::Publisher pub_button;
      ros::Publisher pub_button_color;
      ros::Publisher pub_preset_ui;
      ros::Publisher pub_unset;
      ros::Publisher pub_instruction;
      std::vector<StaticBorder> l_borders;


   public:
      ProjectorInterface(ros::NodeHandle *nh_, std::string name_vb, std::string name_cc, std::string name_border, std::string name_pre, std::string name_un, std::string name_in) :
      // Bind the callback to the action server. False is for thread spinning
      //nh_border_(*n),
      as_(*nh_, name_vb, boost::bind(&ProjectorInterface::executeVirtualButtonsGoal, this, _1), false),
      as_change(*nh_, name_cc, boost::bind(&ProjectorInterface::executeChangeButtonColor, this, _1), false),
      as_border(*nh_, name_border, boost::bind(&ProjectorInterface::executeSafetyBorder, this, _1), false),
      as_preset_ui(*nh_, name_pre, boost::bind(&ProjectorInterface::executePresetUI, this, _1), false),
      as_unset(*nh_, name_un, boost::bind(&ProjectorInterface::executeUnsetUI, this, _1), false),
      as_instruct(*nh_, name_in, boost::bind(&ProjectorInterface::executeInstruction, this, _1), false),
      action_name_button_(name_vb),
      action_name_color_(name_cc),
      action_name_border_(name_border),
      action_name_preset_ui(name_pre),
      action_name_unset(name_un),
      action_name_instruct(name_in)
      {
         //Start prerequisites
         displayed_request_ids.clear();
         pub_button = nh_->advertise<VirtualButtonReference> ("/interfaceUI/openflow/new_button", 1);
         pub_button_color = nh_->advertise<VirtualButtonReference> ("/interfaceUI/openflow/change_button_color", 1);
         pub_preset_ui = nh_->advertise<ProjectorUI>("/interfaceUI/openflow/new_interface",1);
         pub_unset = nh_->advertise<std_msgs::Bool>("/interfaceUI/openflow/unset_projection",1);
         pub_instruction = nh_->advertise<unity_msgs::Instructions>("/interfaceUI/openflow/set_instruction",1);
         // Start the action server
         as_.start();
         as_change.start();
         as_border.start();
         as_preset_ui.start();
         as_unset.start();
         as_instruct.start();
         activate_dynamic_border = false;
         activate_static_border = false;
         add_static_border = false;
         nh_border_.setCallbackQueue(&queue);
         nh_static_border_.setCallbackQueue(&static_border_queue);
         g_spinner.reset(new ros::AsyncSpinner(0, &queue));
         static_border_spinner.reset(new ros::AsyncSpinner(0, &static_border_queue));
      }

      // Destructor
      ~ProjectorInterface(void)
      {
      }

      // Execute action callback (passing the goal via reference)
      void executeVirtualButtonsGoal(const SetVirtualButtonsProjectionGoalConstPtr &goal)
      {
         // monitor time
         ros::Rate r(1);
         bool success = true;
         //send button to inteface
         displayed_request_ids.push_back(goal->request_id);
         VirtualButtonReference msg = fillMsg(goal);
         pub_button.publish(msg);
         sendFeedBackButton(goal->request_id);
         if (as_.isPreemptRequested() || !ros::ok())
         {
            ROS_INFO("%s: Preempted", action_name_button_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
         }
         if(success)
         {
            //result_.displayed_request_ids.clear();
            ROS_INFO("%s: Succeeded", action_name_button_.c_str());
            // set the action state to succeeded
            sendResultButton(goal->request_id);
         }
      }

      void sendFeedBackButton(std::string id_goal)
      {
         //feedback_button_.displayed_request_ids.;
         feedback_button_.feedback.displayed_request_ids = id_goal;
         //feedback_button_.displayed_request_ids = id_goal;
         as_.publishFeedback(feedback_button_.feedback);
      }

      void sendResultButton(std::string id_goal)
      {
         //result_.displayed_request_ids.clear();
         result_.result.displayed_request_ids = id_goal;
         //result_.displayed_request_ids = id_goal;
         as_.setSucceeded(result_.result);
      }

      void executeChangeButtonColor(const SetVirtualButtonChangeColorGoalConstPtr &goal)
      {
         ros::Rate r(1);
         bool success = true;
         //send button to inteface
         VirtualButtonReference msg;
         msg.id = goal->resource_id;
         msg.button_color = goal->button_color;
         pub_button_color.publish(msg);
         sendFeedBackChangeButton(goal->request_id);
         if (as_change.isPreemptRequested() || !ros::ok())
         {
            ROS_INFO("%s: Preempted", action_name_color_.c_str());
            // set the action state to preempted
            as_change.setPreempted();
            success = false;
         }
         if(success)
         {
            //result_.displayed_request_ids.clear();
            ROS_INFO("%s: Succeeded", action_name_color_.c_str());
            // set the action state to succeeded
            sendResultChangeButton(goal->request_id);
         }
      }

      void sendFeedBackChangeButton(std::string id_goal)
      {
         feedback_color_.feedback.displayed_request_ids.clear();
         feedback_color_.feedback.displayed_request_ids.push_back(id_goal);
         as_change.publishFeedback(feedback_color_.feedback);
      }

      void sendResultChangeButton(std::string id_goal)
      {
         result_color_.displayed_request_ids.clear();
         result_color_.displayed_request_ids.push_back(id_goal);
         as_change.setSucceeded(result_color_);
      }

      void executeSafetyBorder(const SetSafetyBorderProjectionGoalConstPtr &goal)
      {
         //ros::Rate r(1);
         bool success = true;
         //send button to inteface
         
         if(goal->border.polygon.points.size() > 1)
         {
            //static border
            std::cout<<"static border\n";
            if(!activate_static_border)
            {
               activate_static_border = true;
               StaticBorder sb(goal->request_id,goal->zone,goal->border,goal->border_topic,goal->border_color,goal->is_filled,goal->thickness,goal->lifetime,goal->track_violations);
               std::thread th_sb(&ProjectorInterface::threadCreateStaticBorder,this,nh_static_border_,sb);
               th_sb.detach();
            }
            else
            {
               StaticBorder sb(goal->request_id,goal->zone,goal->border,goal->border_topic,goal->border_color,goal->is_filled,goal->thickness,goal->lifetime,goal->track_violations);
               l_borders.push_back(sb);
               add_static_border = true;
            }
         }
         else
         {
            if(activate_dynamic_border)
            {
               std::cout<<"Dynamic Border already running\n";
               success = false;
            }
            else
            {
               //dynamic border
               std::cout<<"dynamic border\n";
               activate_dynamic_border = true;
               std::thread th_b(&ProjectorInterface::threadCreateDynamicBorder,this,nh_border_,goal->request_id,goal->zone,goal->border_topic,goal->border_color,goal->is_filled,goal->thickness,goal->lifetime,goal->track_violations);
               th_b.detach();
            }
            
         }
         displayed_ids_borders.push_back(goal->request_id);
         sendFeedBackBorder();
         if (as_border.isPreemptRequested() || !ros::ok() || !success)
         {
            ROS_INFO("%s: Preempted", action_name_border_.c_str());
            // set the action state to preempted
            as_border.setPreempted();
            success = false;
         }
         if(success)
         {
            ROS_INFO("%s: Succeeded", action_name_border_.c_str());
            // set the action state to succeeded
            sendResultBorder();
         }
      }

      void threadCreateDynamicBorder(ros::NodeHandle nh_, std::string r_id, std::string z, std::string b_topic, std_msgs::ColorRGBA b_color, bool filling, int thic, ros::Duration life, bool track)
      {
         DynamicBorder db(&nh_,r_id,z,b_topic,b_color,filling,thic,life,track);
         g_spinner->start();
         while(activate_dynamic_border)
         {
            queue.callAvailable();  
            
            //g_spinner->start();
         }
         g_spinner->stop();
         std::cout<<"Terminating Dynamic Border\n";
      }

      void threadCreateStaticBorder(ros::NodeHandle nh_, StaticBorder sb)
      {
         StaticBorderManager sbm(&nh_,sb);
         static_border_spinner->start();
         while(activate_static_border)
         {
            if(add_static_border)
            {
               std::cout<<"add new border ! \n";
               sbm.addBorder(l_borders[0]);
               add_static_border = false;
               l_borders.clear();
            }
            static_border_queue.callAvailable(); 
            //std::cout<<"spinner\n"; 
         }
         static_border_spinner->stop();
         std::cout<<"Terminating Static Border\n";
      }

      void sendFeedBackBorder()
      {
         feedback_border_.feedback.displayed_request_ids.clear();
         for(std::string i : displayed_ids_borders)
         {
            feedback_border_.feedback.displayed_request_ids.push_back(i);
         }
         as_border.publishFeedback(feedback_border_.feedback);
      }

      void sendResultBorder()
      {
         result_border_.displayed_request_ids.clear();
         for(std::string i : displayed_ids_borders)
         {
            result_border_.displayed_request_ids.push_back(i);
         }
         as_border.setSucceeded(result_border_);
      }

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

      void sendFeedBackPresetUI()
      {
         feedback_preset_ui.feedback.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            feedback_preset_ui.feedback.displayed_request_ids.push_back(i);
         }
         as_preset_ui.publishFeedback(feedback_preset_ui.feedback);
      }

      void sendResultPresetUI()
      {
         result_preset_ui.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            result_preset_ui.displayed_request_ids.push_back(i);
         }
         as_preset_ui.setSucceeded(result_preset_ui);
      }

      void executeUnsetUI(const UnsetProjectionGoalConstPtr &goal)
      {
         ros::Rate r(1);
         bool success = true;
         displayed_request_ids.push_back(goal->request_id);
         //unset border
         activate_dynamic_border = false;
         activate_static_border = false;
         //unset projection
         std_msgs::Bool msg;
         msg.data = true;
         pub_unset.publish(msg);
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

      void sendFeedBackUnset()
      {
         feedback_unset_.feedback.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            feedback_unset_.feedback.displayed_request_ids.push_back(i);
         }
         as_unset.publishFeedback(feedback_unset_.feedback);
      }

      void sendResultUnset()
      {
         result_unset_.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            result_unset_.displayed_request_ids.push_back(i);
         }
         as_unset.setSucceeded(result_unset_);
      }

      void executeInstruction(const SetInstructionsProjectionGoalConstPtr &goal)
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

      void sendFeedBackInstruction()
      {
         feedback_instruct_.feedback.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            feedback_instruct_.feedback.displayed_request_ids.push_back(i);
         }
         as_instruct.publishFeedback(feedback_instruct_.feedback);
      }

      void sendResultInstruction()
      {
         result_instruct_.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            result_instruct_.displayed_request_ids.push_back(i);
         }
         as_instruct.setSucceeded(result_instruct_);
      }

      VirtualButtonReference fillMsg(const SetVirtualButtonsProjectionGoalConstPtr &goal)
      {
         VirtualButtonReference msg;
         msg.id = goal->virtual_button.id;
         msg.zone = goal->zone;
         msg.name = goal->virtual_button.name;
         msg.description = goal->virtual_button.description;
         msg.text = goal->virtual_button.text;
         msg.button_color = goal->virtual_button.button_color;
         msg.text_color = goal->virtual_button.text_color;
         msg.center = goal->virtual_button.center;
         msg.radius = goal->virtual_button.radius;
         msg.lifetime = goal->virtual_button.lifetime;
         msg.hidden = goal->virtual_button.hidden;

         return msg;
      }
};


int main(int argc, char** argv)
{
   ros::init(argc, argv, "projector_interface_srv");
   boost::shared_ptr<ros::AsyncSpinner> g_spinner;
   ros::CallbackQueue queue;
   ros::NodeHandle nh;
   //ros::NodeHandle nh_border;
   nh.setCallbackQueue(&queue);
   g_spinner.reset(new ros::AsyncSpinner(0, &queue));
   
   // Create an action server object and spin ROS
   ProjectorInterface srv(&nh,"execution/projector_interface/integration/actions/set_virtual_buttons_projection",
                        "execution/projector_interface/integration/actions/set_virtual_button_change_color",
                        "execution/projector_interface/integration/actions/set_safety_border_projection",
                        "execution/projector_interface/integration/actions/set_preset_ui_projection",
                        "execution/projector_interface/integration/actions/unset_projection",
                        "execution/projector_interface/integration/actions/set_instructions_projection");
   
   g_spinner->start();
   ros::waitForShutdown();
   

   return 0;
}