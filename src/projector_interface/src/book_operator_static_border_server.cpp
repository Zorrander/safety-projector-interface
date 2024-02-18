/*
The class is the openflow server that will receives all the actions coming from openflow
*/

#include "projector_interface/book_operator_static_border_server.h"


using namespace integration;

    BookOeratorStaticBorderServer(ros::NodeHandle* nh_, std::string name_book_operator) :
        // Bind the callback to the action server. False is for thread spinning
        //nh_border_(*n),
        as_book_operator(*nh_, name_book_operator, boost::bind(&BookOeratorStaticBorderServer::executeBookOperator, this, _1), false),
        action_name_book_operator(name_book_operator)
    {
        border_operator_booked.clear();
        as_book_operator.start();
    }
      
    //book operator border
    void executeBookOperator(const BookOperatorStaticBorderGoalConstPtr& goal)
    {
        border_operator_booked.push_back(goal->id);
        book_operator_border = true;
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
    void sendFeedbackBookOperator(std::string req_id)
    {
        feedback_book_operator_.feedback.displayed_request_ids.clear();
        feedback_book_operator_.feedback.displayed_request_ids.push_back(req_id);
        as_book_operator.publishFeedback(feedback_book_operator_.feedback);
    }
    //send result
    void sendResultBookOperator(std::string req_id)
    {
        result_book_operator_.displayed_request_ids.clear();
        result_book_operator_.displayed_request_ids.push_back(req_id);
        as_book_operator.setSucceeded(result_book_operator_);
    }