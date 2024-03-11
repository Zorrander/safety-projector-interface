#ifndef ButtonProjectionServer_H
#define ButtonProjectionServer_H

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <integration/SetVirtualButtonsProjectionAction.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <integration/VirtualButtonReference.h>

#include <string>

using namespace integration;

class ButtonProjectionServer
{
protected:
    
    actionlib::SimpleActionServer<SetVirtualButtonsProjectionAction> as_;

    std::string action_name_button_;
    std::vector<std::string> displayed_request_ids;
    
    // create messages that are used to published feedback/result
    SetVirtualButtonsProjectionActionFeedback feedback_button_;
    SetVirtualButtonsProjectionActionResult result_;

    ros::Publisher pub_button;

public:
    ButtonProjectionServer(ros::NodeHandle* nh_, std::string name_vb);
    // create a virtual button
    void executeVirtualButtonsGoal(const SetVirtualButtonsProjectionGoalConstPtr& goal);
    //send feedback
    void sendFeedBackButton(std::string id_goal);
    //send result
    void sendResultButton(std::string id_goal);

    //fill ros msg to send button
    VirtualButtonReference fillMsg(const SetVirtualButtonsProjectionGoalConstPtr& goal);
};
#endif

