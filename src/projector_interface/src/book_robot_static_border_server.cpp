
#include "projection_interface/book_robot_static_border.h"


using namespace integration;


    BookRobotStaticBorderServer(ros::NodeHandle* nh_, std::string name_book_robot) :
        as_book_robot(*nh_, name_book_robot, boost::bind(&ProjectorInterface::executeBookRobot, this, _1), false),
        action_name_book_robot(name_book_robot)
    {
        border_robot_booked.clear();
      
        as_book_robot.start();
    }

    
    //book robot border
    void executeBookRobot(const BookRobotStaticBorderGoalConstPtr& goal)
    {
        border_robot_booked.push_back(goal->id);
        book_robot_border = true;
        bool success = true;
        sendFeedbackBookRobot(goal->request_id);
        if (as_book_robot.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_book_robot.c_str());
            // set the action state to preempted
            as_book_robot.setPreempted();
            success = false;
        }
        if (success)
        {
            ROS_INFO("%s: Succeeded", action_name_book_robot.c_str());
            // set the action state to succeeded
            sendResultBookRobot(goal->request_id);
        }
    }
    //send feedback
    void sendFeedbackBookRobot(std::string req_id)
    {
        feedback_book_robot_.feedback.displayed_request_ids.clear();
        feedback_book_robot_.feedback.displayed_request_ids.push_back(req_id);
        as_book_robot.publishFeedback(feedback_book_robot_.feedback);
    }
    //send result
    void sendResultBookRobot(std::string req_id)
    {
        result_book_robot_.displayed_request_ids.clear();
        result_book_robot_.displayed_request_ids.push_back(req_id);
        as_book_robot.setSucceeded(result_book_robot_);
    }
   

