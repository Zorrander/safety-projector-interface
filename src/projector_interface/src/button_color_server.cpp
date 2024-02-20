
#include "projector_interface/button_color_server.h"


using namespace integration;


    ButtonColorServer(ros::NodeHandle* nh_, std::string name_cc) :

        as_change(*nh_, name_cc, boost::bind(&ButtonColorServer::executeChangeButtonColor, this, _1), false),

        action_name_color_(name_cc)

    {
        pub_button_color = nh_->advertise<VirtualButtonReference>("/interfaceUI/openflow/change_button_color", 1);
        
        as_change.start();
    }

    //change button color
    void executeChangeButtonColor(const SetVirtualButtonChangeColorGoalConstPtr& goal)
    {
        ros::Rate r(1);
        bool success = true;
        //send button to inteface
        VirtualButtonReference msg;
        msg.id = goal->resource_id;
        msg.button_color = goal->button_color;
        pub_button_color.publish(msg);
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
    void sendFeedBackChangeButton(std::string id_goal)
    {
        feedback_color_.feedback.displayed_request_ids.clear();
        feedback_color_.feedback.displayed_request_ids.push_back(id_goal);
        as_change.publishFeedback(feedback_color_.feedback);
    }
    //send result
    void sendResultChangeButton(std::string id_goal)
    {
        result_color_.displayed_request_ids.clear();
        result_color_.displayed_request_ids.push_back(id_goal);
        as_change.setSucceeded(result_color_);
    }
