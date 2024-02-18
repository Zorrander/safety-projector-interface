/*
The class is the openflow server that will receives all the actions coming from openflow
*/
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/SetVirtualButtonsProjectionAction.h>
#include <integration/SetVirtualButtonChangeColorAction.h>
#include <integration/SetSafetyBorderProjectionAction.h>
#include <integration/SetPresetUIProjectionAction.h>
#include <integration/SetLayoutStaticBordersAction.h>
#include <integration/UnsetProjectionAction.h>
#include <integration/SetInstructionsProjectionAction.h>
#include <integration/BookRobotStaticBorderAction.h>
#include <integration/ReleaseRobotStaticBorderAction.h>
#include <integration/BookOperatorStaticBorderAction.h>
#include <integration/ReleaseOperatorStaticBorderAction.h>
#include <integration/MoveJointsAction.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <integration/VirtualButtonReference.h>
#include <integration/ProjectorUI.h>
#include <unity_msgs/Instructions.h>
#include "border/DynamicBorder.hpp"
#include "border/StaticBorder.hpp"
#include "border/StaticBorderManager.hpp"


using namespace integration;

class ButtonProjectionServer
{
protected:
    actionlib::SimpleActionServer<SetVirtualButtonsProjectionAction> as_;

    std::string action_name_button_;

    // create messages that are used to published feedback/result
    SetVirtualButtonsProjectionActionFeedback feedback_button_;
    SetVirtualButtonsProjectionActionResult result_;

    ros::Publisher pub_button;

public:

    // create a virtual button
    void executeVirtualButtonsGoal(const SetVirtualButtonsProjectionGoalConstPtr& goal);
    //send feedback
    void sendFeedBackButton(std::string id_goal);
    //send result
    void sendResultButton(std::string id_goal);

    //fill ros msg to send button
    VirtualButtonReference fillMsg(const SetVirtualButtonsProjectionGoalConstPtr& goal);
}


