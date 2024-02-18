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
#include <thread>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

using namespace integration;


    ProjectorInterface(ros::NodeHandle* nh_, std::string name_vb) :
        as_(*nh_, name_vb, boost::bind(&ProjectorInterface::executeVirtualButtonsGoal, this, _1), false),
        action_name_button_(name_vb)
    {
        pub_button = nh_->advertise<VirtualButtonReference>("/interfaceUI/openflow/new_button", 1);

        as_.start();
    }

    // create a virtual button
    void executeVirtualButtonsGoal(const SetVirtualButtonsProjectionGoalConstPtr& goal)
    {
        // monitor time
        ros::Rate r(1);
        bool success = true;
        //send button to inteface
        displayed_request_ids.push_back(goal->request_id);
        VirtualButtonReference msg = fillMsg(goal);
        pub_button.publish(msg);
        sendFeedBackButton(goal->request_id);
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_button_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
        }
        if (success)
        {
            //result_.displayed_request_ids.clear();
            ROS_INFO("%s: Succeeded", action_name_button_.c_str());
            // set the action state to succeeded
            sendResultButton(goal->request_id);
        }
    }
    //send feedback
    void sendFeedBackButton(std::string id_goal)
    {
        //feedback_button_.displayed_request_ids.;
        feedback_button_.feedback.displayed_request_ids = id_goal;
        //feedback_button_.displayed_request_ids = id_goal;
        as_.publishFeedback(feedback_button_.feedback);
    }
    //send result
    void sendResultButton(std::string id_goal)
    {
        //result_.displayed_request_ids.clear();
        result_.result.displayed_request_ids = id_goal;
        //result_.displayed_request_ids = id_goal;
        as_.setSucceeded(result_.result);
    }
   
    //fill ros msg to send button
    VirtualButtonReference fillMsg(const SetVirtualButtonsProjectionGoalConstPtr& goal)
    {
        VirtualButtonReference msg;
        msg.id = goal->virtual_button.id;
        msg.zone = goal->zone;
        msg.name = goal->virtual_button.name;
        msg.description = goal->virtual_button.description;
        msg.text = goal->virtual_button.text;
        msg.button_color = goal->virtual_button.button_color;
        msg.text_color = goal->virtual_button.text_color;
        msg.center = goal->virtual_button.center;
        msg.radius = goal->virtual_button.radius;
        msg.lifetime = goal->virtual_button.lifetime;
        msg.hidden = goal->virtual_button.hidden;

        return msg;
    }


