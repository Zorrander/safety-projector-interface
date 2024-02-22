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
   
   // Create an action server object and spin ROS
   ButtonProjectionServer srv2(&nh, "execution/projector_interface/integration/actions/set_virtual_buttons_projection");
   ButtonColorServer srv3(&nh, "execution/projector_interface/integration/actions/set_virtual_button_change_color");
   UserInterfaceServer srv4(&nh, "execution/projector_interface/integration/actions/set_preset_ui_projection");
   UnsetProjectionServer srv5(&nh, "execution/projector_interface/integration/actions/unset_projection");
   InstructionProjectionServer srv6(&nh, "execution/projector_interface/integration/actions/set_instructions_projection");
   
   g_spinner->start();

   while (ros::ok()) {
      queue.callAvailable();
   }
   
   ros::waitForShutdown();
   

   return 0;
}