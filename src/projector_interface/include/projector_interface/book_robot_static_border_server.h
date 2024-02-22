#ifndef BookRobotStaticBorderServer_H
#define BookRobotStaticBorderServer_H

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <integration/BookRobotStaticBorderAction.h>


#include "border/DynamicBorder.h"
#include "border/StaticBorder.h"
#include "border/StaticBorderManager.h"

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

using namespace integration;




class BookRobotStaticBorderServer
{
protected:

    actionlib::SimpleActionServer<BookRobotStaticBorderAction> as_book_robot;
    std::string action_name_book_robot;
    bool book_robot_border;
    std::vector<StaticBorder> l_borders;
    BookRobotStaticBorderActionFeedback feedback_book_robot_;
    BookRobotStaticBorderResult result_book_robot_;

    std::vector<std::string> border_robot_booked;
    std::vector<std::string> displayed_request_ids;

public:
    BookRobotStaticBorderServer(ros::NodeHandle* nh_, std::string name_book_robot);
    //book robot border
    void executeBookRobot(const BookRobotStaticBorderGoalConstPtr& goal);
    //send feedback
    void sendFeedbackBookRobot(std::string req_id);
    //send result
    void sendResultBookRobot(std::string req_id);

};
#endif