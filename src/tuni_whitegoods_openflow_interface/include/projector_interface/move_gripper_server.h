#ifndef MoveGripperServer_H
#define MoveGripperServer_H


#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <integration/ControlOpenCloseToolAction.h>


#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

using namespace integration;


class MoveGripperServer
{

   public:
      MoveGripperServer(ros::NodeHandle *nh_, std::string name_mg);
      //move the joints of the robot to some positions
      void executeMoveGripper(const ControlOpenCloseToolGoalConstPtr &goal);
      //send feedback
      void sendFeedbackMoveGripper(ActionRequest ar);
      //send result
      void sendResultMoveGripper(ActionRequest ar);
      
   protected:

      actionlib::SimpleActionServer<ControlOpenCloseToolAction> as_move_gripper;
      
      std::string action_name_move_gripper;
      // create messages that are used to published feedback/result
      ControlOpenCloseToolFeedback feedback_mgf;
      ControlOpenCloseToolResult result_mgr;
      ros::Publisher pub_cmd_robot;
};

#endif
