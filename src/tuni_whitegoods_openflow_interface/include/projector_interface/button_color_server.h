#ifndef ButtonColorServer_H
#define ButtonColorServer_H

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <integration/SetVirtualButtonChangeColorAction.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <integration/VirtualButtonReference.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

using namespace integration;


class ButtonColorServer
{
protected:
    
    actionlib::SimpleActionServer<SetVirtualButtonChangeColorAction> as_change;
    std::string action_name_color_;

    SetVirtualButtonChangeColorActionFeedback feedback_color_;
    SetVirtualButtonChangeColorResult result_color_;

    ros::Publisher pub_button_color;

public:
    ButtonColorServer(ros::NodeHandle* nh_, std::string name_cc);

    //change button color
    void executeChangeButtonColor(const SetVirtualButtonChangeColorGoalConstPtr& goal);
    //senf feedback
    void sendFeedBackChangeButton(std::string id_goal);
    //send result
    void sendResultChangeButton(std::string id_goal);

};
#endif