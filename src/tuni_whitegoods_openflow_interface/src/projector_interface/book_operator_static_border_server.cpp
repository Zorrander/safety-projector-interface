#include "projector_interface/book_operator_static_border_server.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/BookOperatorStaticBorderAction.h>


    BookOperatorStaticBorderServer::BookOperatorStaticBorderServer(ros::NodeHandle* nh_, std::string name_book_operator, std::shared_ptr<ProjectorInterfaceController> controller) :
        // Bind the callback to the action server. False is for thread spinning
        //nh_border_(*n),
        as_book_operator(*nh_, name_book_operator, boost::bind(&BookOperatorStaticBorderServer::executeBookOperator, this, _1), false),
        action_name_book_operator(name_book_operator),
        controller(controller)
    {
        displayed_request_ids.clear();

        as_book_operator.start();
        std::cout<<"BookOperatorStaticBorderServer running\n";
    }
      
    //book operator border
    void BookOperatorStaticBorderServer::executeBookOperator(const BookOperatorStaticBorderGoalConstPtr& goal)
    {

        bool success = true;
        sendFeedbackBookOperator(goal->request_id);
        if (as_book_operator.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_book_operator.c_str());
            // set the action state to preempted
            as_book_operator.setPreempted();
            success = false;
        }
        if (success)
        {
            ROS_INFO("%s: Succeeded", action_name_book_operator.c_str());
            // set the action state to succeeded
            sendResultBookOperator(goal->request_id);
        }
    }
    //send feedback
    void BookOperatorStaticBorderServer::sendFeedbackBookOperator(std::string req_id)
    {
        feedback_book_operator_.feedback.displayed_request_ids.clear();
        feedback_book_operator_.feedback.displayed_request_ids.push_back(req_id);
        as_book_operator.publishFeedback(feedback_book_operator_.feedback);
    }
    //send result
    void BookOperatorStaticBorderServer::sendResultBookOperator(std::string req_id)
    {
        result_book_operator_.displayed_request_ids.clear();
        result_book_operator_.displayed_request_ids.push_back(req_id);
        as_book_operator.setSucceeded(result_book_operator_);
    }