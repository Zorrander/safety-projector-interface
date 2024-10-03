#ifndef MoveGripperServer_H
#define MoveGripperServer_H

#include <actionlib/server/simple_action_server.h>
#include <integration/ControlOpenCloseToolAction.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <ur_dashboard_msgs/Load.h>

#include <string>

using namespace integration;

class MoveGripperServer {
 public:
  MoveGripperServer(ros::NodeHandle *nh_, std::string name_mg);
  // move the joints of the robot to some positions
  void executeMoveGripper(const ControlOpenCloseToolGoalConstPtr &goal);
  // send feedback
  void sendFeedbackMoveGripper(ActionRequest ar);
  // send result
  void sendResultMoveGripper(ActionRequest ar);

 protected:
  bool run_dashboard_script(ur_dashboard_msgs::Load srv_req);

  ros::ServiceClient client_load;
  ros::ServiceClient client_play;

  ur_dashboard_msgs::Load srv_req;
  std_srvs::Trigger srv_play;

  actionlib::SimpleActionServer<ControlOpenCloseToolAction> as_move_gripper;

  std::string action_name_move_gripper;
  // create messages that are used to published feedback/result
  ControlOpenCloseToolFeedback feedback_mgf;
  ControlOpenCloseToolResult result_mgr;
  ros::Publisher pub_cmd_robot;
};

#endif
