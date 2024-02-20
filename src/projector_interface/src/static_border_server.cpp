#include "projector_interface/static_borde_server.h"

using namespace integration;


      StaticBorderServer(ros::NodeHandle *nh_, std::string name_bd_manager) :
      // Bind the callback to the action server. False is for thread spinning
      //nh_border_(*n),
      as_border_manager(*nh_, name_bd_manager, boost::bind(&StaticBorderServer::executeSetLayout, this, _1), false),
      action_name_border_manager_(name_bd_manager),
      {
         //Start prerequisites
         displayed_request_ids.clear();
         border_robot_booked.clear();
         border_operator_booked.clear();
         release_border_robot.clear();
         release_border_operator.clear();
         // Start the action server
         as_border_manager.start();
         activate_static_border_manager = false;
         add_static_border = false;
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
