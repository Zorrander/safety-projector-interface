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

struct BorderStatus {
    std::string id_border;
    int status;
};


class BookRobotStaticBorderServer
{
protected:

    actionlib::SimpleActionServer<BookRobotStaticBorderAction> as_book_robot;

    bool book_robot_border;

    BookRobotStaticBorderActionFeedback feedback_book_robot_;
    BookRobotStaticBorderResult result_book_robot_;

    std::vector<std::string> border_robot_booked;
    std::vector<std::string> displayed_request_ids;

public:

    //book robot border
    void executeBookRobot(const BookRobotStaticBorderGoalConstPtr& goal);
    //send feedback
    void sendFeedbackBookRobot(std::string req_id);
    //send result
    void sendResultBookRobot(std::string req_id);

};