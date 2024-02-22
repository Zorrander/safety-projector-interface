#include "projector_interface/book_robot_static_border_server.h"
#include "projector_interface/release_robot_static_border_server.h"
#include "projector_interface/book_operator_static_border_server.h"
#include "projector_interface/release_operator_static_border_server.h"

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>


int main(int argc, char** argv)
{
   ros::init(argc, argv, "border_booking_node");
   boost::shared_ptr<ros::AsyncSpinner> g_spinner;
   ros::CallbackQueue queue;
   ros::NodeHandle nh;
   //ros::NodeHandle nh_border;
   nh.setCallbackQueue(&queue);
   g_spinner.reset(new ros::AsyncSpinner(0, &queue));
   
   // Create an action server object and spin ROS
   BookRobotStaticBorderServer srv2(&nh, "execution/projector_interface/integration/actions/set_virtual_buttons_projection");
   ReleaseRobotStaticBorderServer srv3(&nh, "execution/projector_interface/integration/actions/set_virtual_button_change_color");
   BookOperatorStaticBorderServer srv4(&nh, "execution/projector_interface/integration/actions/set_preset_ui_projection");
   ReleaseOperatorStaticBorderServer srv5(&nh, "execution/projector_interface/integration/actions/unset_projection");
   
   g_spinner->start();

   while (ros::ok()) {
      queue.callAvailable();
   }

   ros::waitForShutdown();
   

   return 0;
}