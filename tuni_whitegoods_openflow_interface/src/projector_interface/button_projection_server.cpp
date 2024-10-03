#include "projector_interface/button_projection_server.h"

#include <actionlib/server/simple_action_server.h>
#include <integration/SetVirtualButtonsProjectionAction.h>
#include <ros/ros.h>

ButtonProjectionServer::ButtonProjectionServer(
    ros::NodeHandle* nh_, std::string name_vb,
    std::shared_ptr<ProjectorInterfaceController>
        projector_interface_controller)
    : as_(*nh_, name_vb,
          boost::bind(&ButtonProjectionServer::executeVirtualButtonsGoal, this,
                      _1),
          false),
      action_name_button_(name_vb),
      controller(projector_interface_controller) {
  as_.start();
  std::cout << "ButtonProjectionServer running \n";
}

// create a virtual button
void ButtonProjectionServer::executeVirtualButtonsGoal(
    const SetVirtualButtonsProjectionGoalConstPtr& goal) {
  controller->addButton(
      goal->request_id, goal->zone, goal->virtual_button.name,
      goal->virtual_button.text, goal->virtual_button.button_color,
      goal->virtual_button.text_color, goal->virtual_button.center,
      goal->virtual_button.radius);
  bool success = true;
  sendFeedBackButton(goal->request_id);
  if (as_.isPreemptRequested() || !ros::ok()) {
    ROS_INFO("%s: Preempted", action_name_button_.c_str());
    // set the action state to preempted
    as_.setPreempted();
    success = false;
  }
  if (success) {
    // result_.displayed_request_ids.clear();
    ROS_INFO("%s: Succeeded", action_name_button_.c_str());
    // set the action state to succeeded
    sendResultButton(goal->request_id);
  }
}
// send feedback
void ButtonProjectionServer::sendFeedBackButton(std::string id_goal) {
  // feedback_button_.displayed_request_ids.;
  feedback_button_.feedback.displayed_request_ids = id_goal;
  // feedback_button_.displayed_request_ids = id_goal;
  as_.publishFeedback(feedback_button_.feedback);
}
// send result
void ButtonProjectionServer::sendResultButton(std::string id_goal) {
  // result_.displayed_request_ids.clear();
  result_.result.displayed_request_ids = id_goal;
  // result_.displayed_request_ids = id_goal;
  as_.setSucceeded(result_.result);
}
