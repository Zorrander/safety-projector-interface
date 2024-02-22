#include "projector_interface/move_robot_server.h"

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>


int main(int argc, char** argv)
{
   ros::init(argc, argv, "move_robot_server");
   boost::shared_ptr<ros::AsyncSpinner> g_spinner;
   ros::CallbackQueue queue;
   ros::NodeHandle nh;
   //ros::NodeHandle nh_border;
   nh.setCallbackQueue(&queue);
   g_spinner.reset(new ros::AsyncSpinner(0, &queue));
   
   // Create an action server object and spin ROS
   MoveRobotServer moveRobotServer(&nh, "execution/projector_interface/integration/actions/set_virtual_buttons_projection");
   
   g_spinner->start();

   while (ros::ok()) {
      queue.callAvailable();
   }

   ros::waitForShutdown();
   

   return 0;
}