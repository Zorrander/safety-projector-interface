#include "projector_interface/static_border_server.h"

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
   
   StaticBorderServer static_border_server(&nh, "execution/projector_interface/integration/actions/set_layout_static_borders");
      
   g_spinner->start();



   ros::waitForShutdown();
   

   return 0;
}