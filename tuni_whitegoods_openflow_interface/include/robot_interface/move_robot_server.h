#ifndef MoveRobotServer_H
#define MoveRobotServer_H

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <integration/MoveJointsAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <string>

using namespace integration;

class MoveRobotServer {
 public:
  MoveRobotServer(ros::NodeHandle *nh_, std::string name_mj);
  // move the joints of the robot to some positions
  void executeMoveJoints(const MoveJointsGoalConstPtr &goal);
  // send feedback
  void sendFeedbackMoveJoints(ActionRequest ar);
  // send result
  void sendResultMoveJoints(ActionRequest ar);

 protected:
  actionlib::SimpleActionServer<MoveJointsAction> as_move_joints;

  std::string action_name_move_joints;
  // create messages that are used to published feedback/result
  MoveJointsActionFeedback feedback_mj;
  MoveJointsActionResult result_mj;
  ros::Publisher pub_cmd_robot;
};

#endif
