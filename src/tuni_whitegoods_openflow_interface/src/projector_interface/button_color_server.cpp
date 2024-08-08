#include "projector_interface/button_color_server.h"
#include "projector_interface_controller.h"
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/SetVirtualButtonChangeColorAction.h>




    ButtonColorServer::ButtonColorServer(ros::NodeHandle* nh_, std::string name_cc, std::shared_ptr<ProjectorInterfaceController> projector_interface_controller) :
        as_change(*nh_, name_cc, boost::bind(&ButtonColorServer::executeChangeButtonColor, this, _1), false),
        action_name_color_(name_cc),
        controller(projector_interface_controller)

    {
        as_change.start();
        std::cout<<"ButtonColorServer running \n";
    }

    //change button color
    void ButtonColorServer::executeChangeButtonColor(const SetVirtualButtonChangeColorGoalConstPtr& goal)
    {
        sendFeedBackChangeButton(goal->request_id);
        if (as_change.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_color_.c_str());
            // set the action state to preempted
            as_change.setPreempted();
            success = false;
        }
        if (success)
        {
            //result_.displayed_request_ids.clear();
            ROS_INFO("%s: Succeeded", action_name_color_.c_str());
            // set the action state to succeeded
            sendResultChangeButton(goal->request_id);
        }
    }
    //senf feedback
    void ButtonColorServer::sendFeedBackChangeButton(std::string id_goal)
    {
        feedback_color_.feedback.displayed_request_ids.clear();
        feedback_color_.feedback.displayed_request_ids.push_back(id_goal);
        as_change.publishFeedback(feedback_color_.feedback);
    }
    //send result
    void ButtonColorServer::sendResultChangeButton(std::string id_goal)
    {
        result_color_.displayed_request_ids.clear();
        result_color_.displayed_request_ids.push_back(id_goal);
        as_change.setSucceeded(result_color_);
    }
