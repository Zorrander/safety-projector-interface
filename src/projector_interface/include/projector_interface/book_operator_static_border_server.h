/*
The class is the openflow server that will receives all the actions coming from openflow
*/
#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <integration/BookOperatorStaticBorderAction.h>
#include "border/DynamicBorder.hpp"
#include "border/StaticBorder.hpp"
#include "border/StaticBorderManager.hpp"
#include <thread>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>


using namespace integration;

struct BorderStatus {
    std::string id_border;
    int status;
};


class BookOperatorStaticBorderServer
{
protected:
    // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    actionlib::SimpleActionServer<BookOperatorStaticBorderAction> as_book_operator;

    std::string action_name_book_operator;
    bool book_operator_border;

    // create messages that are used to published feedback/result
    BookOperatorStaticBorderActionFeedback feedback_book_operator_;
    BookOperatorStaticBorderResult result_book_operator_;

    //attributes to monitor situation
    //monitor active buttons and publish them
    std::vector<std::string> displayed_request_ids;
    std::vector<std::string> border_operator_booked;
    std::vector<BorderStatus> release_border_operator;
    std::vector<StaticBorder> l_borders;


public:

    void executeBookOperator(const BookOperatorStaticBorderGoalConstPtr& goal);

    void sendFeedbackBookOperator(std::string req_id);

    void sendResultBookOperator(std::string req_id);