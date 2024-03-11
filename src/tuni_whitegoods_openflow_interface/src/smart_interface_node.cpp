#include "projector_interface/button_projection_server.h"
#include "projector_interface/button_color_server.h"
#include "projector_interface/instruction_projection_server.h"
#include "projector_interface/user_interface_server.h"
#include "projector_interface/unset_projection_server.h"

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>



int main(int argc, char** argv)
{
   ros::init(argc, argv, "smart_interface_node");
   boost::shared_ptr<ros::AsyncSpinner> g_spinner;
   ros::CallbackQueue queue;
   ros::NodeHandle nh;
   //ros::NodeHandle nh_border;
   nh.setCallbackQueue(&queue);
   g_spinner.reset(new ros::AsyncSpinner(0, &queue));
   
   std::string button_projection_server_name,
               button_color_server_name,
               user_interface_server_name,
               unset_projection_server_name,
               instruction_projection_server_name;

   ros::param::get("button_projection_server_name", button_projection_server_name);
   ros::param::get("button_color_server_name", button_color_server_name);
   ros::param::get("user_interface_server_name", user_interface_server_name);
   ros::param::get("unset_projection_server_name", unset_projection_server_name);
   ros::param::get("instruction_projection_server_name", instruction_projection_server_name);

   // Create an action server object and spin ROS
   ButtonProjectionServer srv2(&nh, button_projection_server_name);
   ButtonColorServer srv3(&nh, button_color_server_name);
   UserInterfaceServer srv4(&nh, user_interface_server_name);
   UnsetProjectionServer srv5(&nh, unset_projection_server_name);
   InstructionProjectionServer srv6(&nh, instruction_projection_server_name);
   
   g_spinner->start();

   while (ros::ok()) {
      queue.callAvailable();
   }
   
   ros::waitForShutdown();
   

   return 0;
}