#include "projector_interface/safety_border_server.h"

using namespace integration;

      SafetyBorderServer(ros::NodeHandle *nh_, std::string name_border) :
      // Bind the callback to the action server. False is for thread spinning
      as_border(*nh_, name_border, boost::bind(&SafetyBorderServer::executeSafetyBorder, this, _1), false),
      action_name_border_(name_border)
      {
         //Start prerequisites
         displayed_request_ids.clear();
         as_border.start();

         //trajectory_ur5 = new Trajectory_action("follow_joint_trajectory", true);
         // wait for action server to come up
         //while(!trajectory_ur5->waitForServer(ros::Duration(5.0))){
         //   ROS_INFO("Waiting for the joint_trajectory_action server");
         //}
         activate_dynamic_border = false;
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

};
