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

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Trajectory_action;

class ProjectorInterface
{
   protected:
      ros::Subscriber js_sub;
      boost::shared_ptr<ros::AsyncSpinner> g_spinner;
      ros::CallbackQueue queue;
      //ros::NodeHandle nh_;
      ros::NodeHandle nh_border_;
      boost::shared_ptr<ros::AsyncSpinner> static_border_manager_spinner;
      ros::CallbackQueue static_border_manager_queue;
      //ros::NodeHandle nh_;
      ros::NodeHandle nh_static_border_manager_;
      // NodeHandle instance must be created before this line. Otherwise strange error occurs.
      actionlib::SimpleActionServer<SetVirtualButtonsProjectionAction> as_;
      actionlib::SimpleActionServer<SetVirtualButtonChangeColorAction> as_change;
      actionlib::SimpleActionServer<SetSafetyBorderProjectionAction> as_border;
      actionlib::SimpleActionServer<SetLayoutStaticBordersAction> as_border_manager;
      actionlib::SimpleActionServer<SetPresetUIProjectionAction> as_preset_ui;
      actionlib::SimpleActionServer<UnsetProjectionAction> as_unset;
      actionlib::SimpleActionServer<SetInstructionsProjectionAction> as_instruct;
      actionlib::SimpleActionServer<BookRobotStaticBorderAction> as_book_robot;
      actionlib::SimpleActionServer<ReleaseRobotStaticBorderAction> as_release_robot;
      actionlib::SimpleActionServer<BookOperatorStaticBorderAction> as_book_operator;
      actionlib::SimpleActionServer<ReleaseOperatorStaticBorderAction> as_release_operator;
      actionlib::SimpleActionServer<MoveJointsAction> as_move_joints;
      
      Trajectory_action *trajectory_ur5;

      std::string action_name_button_;
      std::string action_name_color_;
      std::string action_name_border_;
      std::string action_name_border_manager_;
      std::string action_name_preset_ui;
      std::string action_name_unset;
      std::string action_name_instruct;
      std::string action_name_book_robot;
      std::string action_name_release_robot;
      std::string action_name_book_operator;
      std::string action_name_release_operator;
      std::string action_name_move_joints;
      bool activate_dynamic_border;
      bool activate_static_border_manager;
      bool add_static_border;
      bool book_robot_border;
      bool book_operator_border;
      bool release_robot_border;
      bool release_operator_border;
      //std::vector<> vec_borders;

      // create messages that are used to published feedback/result
      SetVirtualButtonsProjectionActionFeedback feedback_button_;
      SetVirtualButtonsProjectionActionResult result_;
      SetVirtualButtonChangeColorActionFeedback feedback_color_;
      SetVirtualButtonChangeColorResult result_color_;
      SetLayoutStaticBordersActionFeedback feedback_layout;
      SetLayoutStaticBordersResult result_layout;
      SetSafetyBorderProjectionActionFeedback feedback_border_;
      SetSafetyBorderProjectionResult result_border_;
      SetPresetUIProjectionActionFeedback feedback_preset_ui;
      SetPresetUIProjectionResult result_preset_ui;
      UnsetProjectionActionFeedback feedback_unset_;
      UnsetProjectionResult result_unset_;
      SetInstructionsProjectionActionFeedback feedback_instruct_;
      SetInstructionsProjectionResult result_instruct_;
      BookRobotStaticBorderActionFeedback feedback_book_robot_;
      BookRobotStaticBorderResult result_book_robot_;
      ReleaseRobotStaticBorderActionFeedback feedback_release_robot_;
      ReleaseRobotStaticBorderResult result_release_robot_;
      BookOperatorStaticBorderActionFeedback feedback_book_operator_;
      BookOperatorStaticBorderResult result_book_operator_;
      ReleaseOperatorStaticBorderActionFeedback feedback_release_operator_;
      ReleaseOperatorStaticBorderResult result_release_operator;
      MoveJointsActionFeedback feedback_mj;
      MoveJointsActionResult result_mj;
      

      //attributes to monitor situation
      //monitor active buttons and publish them
      std::vector<std::string> displayed_request_ids;
      std::vector<std::string> displayed_ids_borders;
      std::vector<std::string> border_robot_booked;
      std::vector<std::string> border_operator_booked;
      std::vector<BorderStatus> release_border_robot;
      std::vector<BorderStatus> release_border_operator;
      ros::Publisher pub_button;
      ros::Publisher pub_button_color;
      ros::Publisher pub_preset_ui;
      ros::Publisher pub_unset;
      ros::Publisher pub_instruction;
      ros::Publisher pub_cmd_robot;
      std::vector<StaticBorder> l_borders;
      sensor_msgs::JointState js;


   public:
   //The server can receives ros action to perform several tasks. In order of parameters here :
   // create a border manager
   // create a virtual button (for smart interface)
   // change a button color
   // create a border (static of dynamic)
   // create a smart interface
   // destroy (unset) a smart interface
   // create an instruction to display
   // book a robot border
   // release a robot border
   // book an operator border
   // release an operator border
   // movejoints of robot
      ProjectorInterface(ros::NodeHandle *nh_, std::string name_bd_manager, std::string name_vb, std::string name_cc, std::string name_border, 
                         std::string name_pre, std::string name_un, std::string name_in, std::string name_book_robot, std::string name_release_robot, 
                         std::string name_book_operator, std::string name_release_operator, std::string name_mj) :
      // Bind the callback to the action server. False is for thread spinning
      //nh_border_(*n),
      as_border_manager(*nh_, name_bd_manager, boost::bind(&ProjectorInterface::executeSetLayout, this, _1), false),
      as_(*nh_, name_vb, boost::bind(&ProjectorInterface::executeVirtualButtonsGoal, this, _1), false),
      as_change(*nh_, name_cc, boost::bind(&ProjectorInterface::executeChangeButtonColor, this, _1), false),
      as_border(*nh_, name_border, boost::bind(&ProjectorInterface::executeSafetyBorder, this, _1), false),
      as_preset_ui(*nh_, name_pre, boost::bind(&ProjectorInterface::executePresetUI, this, _1), false),
      as_unset(*nh_, name_un, boost::bind(&ProjectorInterface::executeUnsetUI, this, _1), false),
      as_instruct(*nh_, name_in, boost::bind(&ProjectorInterface::executeInstruction, this, _1), false),
      as_book_robot(*nh_, name_book_robot, boost::bind(&ProjectorInterface::executeBookRobot, this, _1), false),
      as_release_robot(*nh_, name_release_robot, boost::bind(&ProjectorInterface::executeReleaseRobot, this, _1), false),
      as_book_operator(*nh_, name_book_operator, boost::bind(&ProjectorInterface::executeBookOperator, this, _1), false),
      as_release_operator(*nh_, name_release_operator, boost::bind(&ProjectorInterface::executeReleaseOperator, this, _1), false),
      as_move_joints(*nh_, name_mj, boost::bind(&ProjectorInterface::executeMoveJoints, this, _1), false),
      action_name_border_manager_(name_bd_manager),
      action_name_button_(name_vb),
      action_name_color_(name_cc),
      action_name_border_(name_border),
      action_name_preset_ui(name_pre),
      action_name_unset(name_un),
      action_name_instruct(name_in),
      action_name_book_robot(name_book_robot),
      action_name_release_robot(name_release_robot),
      action_name_book_operator(name_book_operator),
      action_name_release_operator(name_release_operator),
      action_name_move_joints(name_mj)
      {
         //Start prerequisites
         displayed_request_ids.clear();
         //js_sub = nh_->subscribe("/joint_states", 1,&ProjectorInterface::callbackJointStates, this);
         pub_button = nh_->advertise<VirtualButtonReference> ("/interfaceUI/openflow/new_button", 1);
         pub_button_color = nh_->advertise<VirtualButtonReference> ("/interfaceUI/openflow/change_button_color", 1);
         pub_preset_ui = nh_->advertise<ProjectorUI>("/interfaceUI/openflow/new_interface",1);
         pub_unset = nh_->advertise<std_msgs::Bool>("/interfaceUI/openflow/unset_projection",1);
         pub_instruction = nh_->advertise<unity_msgs::Instructions>("/interfaceUI/openflow/set_instruction",1);
         pub_cmd_robot = nh_->advertise<std_msgs::String>("/ur_hardware_interface/script_command",1);
         border_robot_booked.clear();
         border_operator_booked.clear();
         release_border_robot.clear();
         release_border_operator.clear();
         // Start the action server
         as_border_manager.start();
         as_.start();
         as_change.start();
         as_border.start();
         as_preset_ui.start();
         as_unset.start();
         as_instruct.start();
         as_book_robot.start();
         as_release_robot.start();
         as_book_operator.start();
         as_release_operator.start();
         as_move_joints.start();
         //trajectory_ur5 = new Trajectory_action("follow_joint_trajectory", true);
         // wait for action server to come up
         //while(!trajectory_ur5->waitForServer(ros::Duration(5.0))){
         //   ROS_INFO("Waiting for the joint_trajectory_action server");
         //}
         activate_dynamic_border = false;
         activate_static_border_manager = false;
         add_static_border = false;
         nh_border_.setCallbackQueue(&queue);
         nh_static_border_manager_.setCallbackQueue(&static_border_manager_queue);
         g_spinner.reset(new ros::AsyncSpinner(0, &queue));
         static_border_manager_spinner.reset(new ros::AsyncSpinner(0, &static_border_manager_queue));
         js.name.resize(6);
         js.position.resize(6);
         js.effort.resize(6);
         js.velocity.resize(6);
      }

      // Destructor
      ~ProjectorInterface(void)
      {
         delete trajectory_ur5;
      }

      void callbackJointStates(const sensor_msgs::JointStateConstPtr& state)
      {
         js.name.clear();
         js.effort.clear();
         js.position.clear();
         js.velocity.clear();
         for(int i = 0; i < state->name.size(); i++)
         {
            js.name.push_back(state->name[i]);
            js.position.push_back(state->position[i]);
            js.velocity.push_back(state->velocity[i]);
            js.effort.push_back(state->effort[i]);
         }
      }

      // create a virtual button
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
      //send feedback
      void sendFeedBackButton(std::string id_goal)
      {
         //feedback_button_.displayed_request_ids.;
         feedback_button_.feedback.displayed_request_ids = id_goal;
         //feedback_button_.displayed_request_ids = id_goal;
         as_.publishFeedback(feedback_button_.feedback);
      }
      //send result
      void sendResultButton(std::string id_goal)
      {
         //result_.displayed_request_ids.clear();
         result_.result.displayed_request_ids = id_goal;
         //result_.displayed_request_ids = id_goal;
         as_.setSucceeded(result_.result);
      }
      //change button color
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
      //senf feedback
      void sendFeedBackChangeButton(std::string id_goal)
      {
         feedback_color_.feedback.displayed_request_ids.clear();
         feedback_color_.feedback.displayed_request_ids.push_back(id_goal);
         as_change.publishFeedback(feedback_color_.feedback);
      }
      //send result
      void sendResultChangeButton(std::string id_goal)
      {
         result_color_.displayed_request_ids.clear();
         result_color_.displayed_request_ids.push_back(id_goal);
         as_change.setSucceeded(result_color_);
      }
      //create a border layout. This will launch a border manager with a thread.
      //The tread keeps the border manager running and ready to create or book borders.
      void executeSetLayout(const SetLayoutStaticBordersGoalConstPtr &goal)
      {
         bool success = true;
         if(!activate_static_border_manager)
         {
            activate_static_border_manager = true;
            //start thread.
            std::thread th_sbm(&ProjectorInterface::threadCreateStaticBorderManager,this,nh_static_border_manager_, goal->size_rows, goal->size_cols, goal->safety_factor, goal->book_adjacent, goal->status_booked, goal->status_free, goal->status_operator);
            th_sbm.detach();
         }
         displayed_ids_borders.push_back(goal->request_id);
         sendFeedbackSetLayout(goal->request_id);
         if (as_border_manager.isPreemptRequested() || !ros::ok() || !success)
         {
            ROS_INFO("%s: Preempted", action_name_border_manager_.c_str());
            // set the action state to preempted
            as_border_manager.setPreempted();
            success = false;
         }
         if(success)
         {
            ROS_INFO("%s: Succeeded", action_name_border_manager_.c_str());
            // set the action state to succeeded
            sendResultSetLayout(goal->request_id);
         }
      }
      //send feedback
      void sendFeedbackSetLayout(std::string id_goal)
      {
         feedback_layout.feedback.displayed_request_ids.clear();
         feedback_layout.feedback.displayed_request_ids.push_back(id_goal);
         as_border_manager.publishFeedback(feedback_layout.feedback);
      }
      //send result
      void sendResultSetLayout(std::string id_goal)
      {
         result_layout.displayed_request_ids.clear();
         result_layout.displayed_request_ids.push_back(id_goal);
         as_border_manager.setSucceeded(result_layout);
      }
      //thread for the border manager
      void threadCreateStaticBorderManager(ros::NodeHandle nh_, int rows, int cols, float sf_factor, bool adjacent, std_msgs::ColorRGBA status_booked, std_msgs::ColorRGBA status_free, std_msgs::ColorRGBA status_operator)
      {
         StaticBorderManager sbm(&nh_,rows, cols, sf_factor, adjacent, status_booked, status_free, status_operator);
         static_border_manager_spinner->start();
         while(activate_static_border_manager)
         {
            //create static border
            if(add_static_border)
            {
               std::cout<<"add new border ! \n";
               sbm.addBorder(l_borders[0]);
               add_static_border = false;
               l_borders.clear();
            }
            //book robot border
            if(book_robot_border)
            {
               for(std::string s : border_robot_booked)
               {
                  sbm.bookBorderRobot(s);
                  std::cout<<"Border Robot BOOKED \n";
               }
               book_robot_border = false;
               border_robot_booked.clear();
            }
            //release robot border
            if(release_robot_border)
            {
               for(BorderStatus sb : release_border_robot)
               {
                  sbm.releaseRobotBorder(sb.id_border,sb.status);
                  std::cout<<"Border robot released \n";
               }
               release_robot_border = false;
               release_border_robot.clear();
            }
            //book operator border
            if(book_operator_border)
            {
               for(std::string s : border_operator_booked)
               {
                  sbm.bookBorderOperator(s);
                  std::cout<<"Border Operator BOOKED \n";
               }
               book_operator_border = false;
               border_operator_booked.clear();
            }
            //release operator border
            if(release_operator_border)
            {
               for(BorderStatus sb : release_border_operator)
               {
                  sbm.releaseOperatorBorder(sb.id_border,sb.status);
                  std::cout<<"Border Operator released \n";
               }
               release_operator_border = false;
               release_border_operator.clear();
            }
            static_border_manager_queue.callAvailable(); 
            //std::cout<<"spinner\n"; 
         }
         static_border_manager_spinner->stop();
         std::cout<<"Terminating Static Border Manager\n";
      }
      //create a static or dynamic border. For a static border, it just pass the information to the already running thread.
      // For the dynamic border, it starts a dynamic border thread.
      void executeSafetyBorder(const SetSafetyBorderProjectionGoalConstPtr &goal)
      {
         //ros::Rate r(1);
         bool success = true;
         //send button to inteface
         
         if(goal->border.polygon.points.size() > 1)
         {
            //static border
            std::cout<<"adding static border...\n";
            if(activate_static_border_manager)
            {
               StaticBorder sb(goal->request_id,goal->zone, goal->position_row, goal->position_col,goal->border,goal->border_topic,goal->border_color,goal->is_filled,goal->thickness,goal->lifetime,goal->track_violations);
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
               //start thread
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
      //thread to create fynamic border
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
      //send feedback
      void sendFeedBackBorder()
      {
         feedback_border_.feedback.displayed_request_ids.clear();
         for(std::string i : displayed_ids_borders)
         {
            feedback_border_.feedback.displayed_request_ids.push_back(i);
         }
         as_border.publishFeedback(feedback_border_.feedback);
      }
      //send result
      void sendResultBorder()
      {
         result_border_.displayed_request_ids.clear();
         for(std::string i : displayed_ids_borders)
         {
            result_border_.displayed_request_ids.push_back(i);
         }
         as_border.setSucceeded(result_border_);
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
      //unset the smart interface
      void executeUnsetUI(const UnsetProjectionGoalConstPtr &goal)
      {
         ros::Rate r(1);
         bool success = true;
         displayed_request_ids.push_back(goal->request_id);
         //unset border
         activate_dynamic_border = false;
         activate_static_border_manager = false;
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
      //send feedback
      void sendFeedBackUnset()
      {
         feedback_unset_.feedback.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            feedback_unset_.feedback.displayed_request_ids.push_back(i);
         }
         as_unset.publishFeedback(feedback_unset_.feedback);
      }
      //send result
      void sendResultUnset()
      {
         result_unset_.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            result_unset_.displayed_request_ids.push_back(i);
         }
         as_unset.setSucceeded(result_unset_);
      }
      //send instruction to be written on the interface
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
      //send feedback
      void sendFeedBackInstruction()
      {
         feedback_instruct_.feedback.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            feedback_instruct_.feedback.displayed_request_ids.push_back(i);
         }
         as_instruct.publishFeedback(feedback_instruct_.feedback);
      }
      //send result
      void sendResultInstruction()
      {
         result_instruct_.displayed_request_ids.clear();
         for(std::string i : displayed_request_ids)
         {
            result_instruct_.displayed_request_ids.push_back(i);
         }
         as_instruct.setSucceeded(result_instruct_);
      }
      //fill ros msg to send button
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
      //book robot border
      void executeBookRobot(const BookRobotStaticBorderGoalConstPtr &goal)
      {
         border_robot_booked.push_back(goal->id);
         book_robot_border = true;
         bool success = true;
         sendFeedbackBookRobot(goal->request_id);
         if (as_book_robot.isPreemptRequested() || !ros::ok())
         {
            ROS_INFO("%s: Preempted", action_name_book_robot.c_str());
            // set the action state to preempted
            as_book_robot.setPreempted();
            success = false;
         }
         if(success)
         {
            ROS_INFO("%s: Succeeded", action_name_book_robot.c_str());
            // set the action state to succeeded
            sendResultBookRobot(goal->request_id);
         }
      }
      //send feedback
      void sendFeedbackBookRobot(std::string req_id)
      {
         feedback_book_robot_.feedback.displayed_request_ids.clear();
         feedback_book_robot_.feedback.displayed_request_ids.push_back(req_id);
         as_book_robot.publishFeedback(feedback_book_robot_.feedback);
      }
      //send result
      void sendResultBookRobot(std::string req_id)
      {
         result_book_robot_.displayed_request_ids.clear();
         result_book_robot_.displayed_request_ids.push_back(req_id);
         as_book_robot.setSucceeded(result_book_robot_);
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
      //book operator border
      void executeBookOperator(const BookOperatorStaticBorderGoalConstPtr &goal)
      {
         border_operator_booked.push_back(goal->id);
         book_operator_border = true;
         bool success = true;
         sendFeedbackBookOperator(goal->request_id);
         if (as_book_operator.isPreemptRequested() || !ros::ok())
         {
            ROS_INFO("%s: Preempted", action_name_book_operator.c_str());
            // set the action state to preempted
            as_book_operator.setPreempted();
            success = false;
         }
         if(success)
         {
            ROS_INFO("%s: Succeeded", action_name_book_operator.c_str());
            // set the action state to succeeded
            sendResultBookOperator(goal->request_id);
         }
      }
      //send feedback
      void sendFeedbackBookOperator(std::string req_id)
      {
         feedback_book_operator_.feedback.displayed_request_ids.clear();
         feedback_book_operator_.feedback.displayed_request_ids.push_back(req_id);
         as_book_operator.publishFeedback(feedback_book_operator_.feedback);
      }
      //send result
      void sendResultBookOperator(std::string req_id)
      {
         result_book_operator_.displayed_request_ids.clear();
         result_book_operator_.displayed_request_ids.push_back(req_id);
         as_book_operator.setSucceeded(result_book_operator_);
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
      //move the joints of the robot to some positions
      void executeMoveJoints(const MoveJointsGoalConstPtr &goal)
      {
         control_msgs::FollowJointTrajectoryGoal goal_ur5;
         goal_ur5.trajectory = goal->trajectory;
         std::string cmd = "movej(["+to_string(goal->trajectory.points[0].positions[0])+","+to_string(goal->trajectory.points[0].positions[1])+","+to_string(goal->trajectory.points[0].positions[2])+","+to_string(goal->trajectory.points[0].positions[3])+","+to_string(goal->trajectory.points[0].positions[4])+","+to_string(goal->trajectory.points[0].positions[5])+"], a=0.3, v=0.13)";
         std_msgs::String command;
         command.data = cmd;
         pub_cmd_robot.publish(command);
         ros::Duration(7.0).sleep();
         bool success = true;
         sendFeedbackMoveJoints(goal->action_request);
         if (as_move_joints.isPreemptRequested() || !ros::ok())
         {
            ROS_INFO("%s: Preempted", action_name_move_joints.c_str());
            // set the action state to preempted
            as_move_joints.setPreempted();
            success = false;
         }
         if(success)
         {
            ROS_INFO("%s: Succeeded", action_name_move_joints.c_str());
            // set the action state to succeeded
            sendResultMoveJoints(goal->action_request);
         }
      }
      //send feedback
      void sendFeedbackMoveJoints(ActionRequest ar)
      {
         MoveJointsFeedback af;
         af.action_feedback.action_id = ar.action_id;
         af.action_feedback.millis_passed = ar.expected_duration_millis;
         as_move_joints.publishFeedback(af);
      }
      //send result
      void sendResultMoveJoints(ActionRequest ar)
      {
         MoveJointsResult mjr;
         mjr.action_result.action_id = ar.action_id;
         mjr.action_result.millis_passed = ar.expected_duration_millis;
         as_move_joints.setSucceeded(mjr);
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
   ProjectorInterface srv(&nh, "execution/projector_interface/integration/actions/set_layout_static_borders",
                        "execution/projector_interface/integration/actions/set_virtual_buttons_projection",
                        "execution/projector_interface/integration/actions/set_virtual_button_change_color",
                        "execution/projector_interface/integration/actions/set_safety_border_projection",
                        "execution/projector_interface/integration/actions/set_preset_ui_projection",
                        "execution/projector_interface/integration/actions/unset_projection",
                        "execution/projector_interface/integration/actions/set_instructions_projection",
                        "execution/projector_interface/integration/actions/set_book_robot_static_border",
                        "execution/projector_interface/integration/actions/set_release_robot_static_border",
                        "execution/projector_interface/integration/actions/set_book_operator_static_border",
                        "execution/projector_interface/integration/actions/set_release_operator_static_border",
                        "execution/cobot/integration/actions/move_arm_joint");
   
   g_spinner->start();
   ros::waitForShutdown();
   

   return 0;
}