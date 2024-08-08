#include "projector_interface/button_projection_server.h"
#include "projector_interface_controller.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/SetVirtualButtonsProjectionAction.h>


    ButtonProjectionServer::ButtonProjectionServer(ros::NodeHandle* nh_, std::string name_vb, std::shared_ptr<ProjectorInterfaceController> projector_interface_controller) :
        as_(*nh_, name_vb, boost::bind(&ButtonProjectionServer::executeVirtualButtonsGoal, this, _1), false),
        action_name_button_(name_vb),
        controller(projector_interface_controller)
    {
        pub_button = nh_->advertise<VirtualButtonReference>("/interfaceUI/openflow/new_button", 1);
        as_.start();
        std::cout<<"ButtonProjectionServer running \n";
    }

    // create a virtual button
    void ButtonProjectionServer::executeVirtualButtonsGoal(const SetVirtualButtonsProjectionGoalConstPtr& goal)
    {
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
    void ButtonProjectionServer::sendFeedBackButton(std::string id_goal)
    {
        //feedback_button_.displayed_request_ids.;
        feedback_button_.feedback.displayed_request_ids = id_goal;
        //feedback_button_.displayed_request_ids = id_goal;
        as_.publishFeedback(feedback_button_.feedback);
    }
    //send result
    void ButtonProjectionServer::sendResultButton(std::string id_goal)
    {
        //result_.displayed_request_ids.clear();
        result_.result.displayed_request_ids = id_goal;
        //result_.displayed_request_ids = id_goal;
        as_.setSucceeded(result_.result);
    }
   
    //fill ros msg to send button
    VirtualButtonReference ButtonProjectionServer::fillMsg(const SetVirtualButtonsProjectionGoalConstPtr& goal)
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


