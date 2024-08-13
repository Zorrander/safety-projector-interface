#include "projector_interface/book_robot_static_border_server.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/BookRobotStaticBorderAction.h>


    BookRobotStaticBorderServer::BookRobotStaticBorderServer(ros::NodeHandle* nh_, std::string name_book_robot, std::shared_ptr<ProjectorInterfaceController> projector_interface_controller) :
        as_book_robot(*nh_, name_book_robot, boost::bind(&BookRobotStaticBorderServer::executeBookRobot, this, _1), false),
        action_name_book_robot(name_book_robot),
        controller(projector_interface_controller)
    {
        displayed_request_ids.clear();      
        as_book_robot.start();
        std::cout<<"BookRobotStaticBorderServer running\n";
    }

    
    //book robot border
    void BookRobotStaticBorderServer::executeBookRobot(const BookRobotStaticBorderGoalConstPtr& goal)
    {
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
    void BookRobotStaticBorderServer::sendFeedbackBookRobot(std::string req_id)
    {
        feedback_book_robot_.feedback.displayed_request_ids.clear();
        feedback_book_robot_.feedback.displayed_request_ids.push_back(req_id);
        as_book_robot.publishFeedback(feedback_book_robot_.feedback);
    }
    //send result
    void BookRobotStaticBorderServer::sendResultBookRobot(std::string req_id)
    {
        result_book_robot_.displayed_request_ids.clear();
        result_book_robot_.displayed_request_ids.push_back(req_id);
        as_book_robot.setSucceeded(result_book_robot_);
    }
   

