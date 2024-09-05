#include "robot_interface/move_robot_server.h"
#include "robot_interface/move_gripper_server.h"

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <integration/MoveJointsAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>


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
   MoveRobotServer moveRobotServer(&nh, "execution/ur/integration/actions/move_arm_joint");
   MoveGripperServer moveGripperServer(&nh, "execution/gripper/integration/actions/control_gripper");

   g_spinner->start();

   while (ros::ok()) {
      queue.callAvailable();
   }

   ros::spin();
   

   return 0;
}