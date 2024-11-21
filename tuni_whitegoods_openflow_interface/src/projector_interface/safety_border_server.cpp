#include "projector_interface/safety_border_server.h"

#include <actionlib/server/simple_action_server.h>
#include <integration/SetSafetyBorderProjectionAction.h>
#include <ros/ros.h>

SafetyBorderServer::SafetyBorderServer(
    ros::NodeHandle *nh_, std::string name_border,
    std::shared_ptr<ProjectorInterfaceController>
        projector_interface_controller)
    : as_border(*nh_, name_border,
                boost::bind(&SafetyBorderServer::executeSafetyBorder, this, _1),
                false),
      action_name_border_(name_border),
      controller(projector_interface_controller) {
  as_border.start();
  std::cout << "SafetyBorderServer running\n";
}

void SafetyBorderServer::executeSafetyBorder(
    const SetSafetyBorderProjectionGoalConstPtr &goal) {
  bool success = true;

  if (goal->zone == "cell_left" || goal->zone == "cell_right") {
    for (auto &zone : controller->model_->getDisplayAreas()) {
      if (zone->name == goal->zone) {
        ROS_INFO("New color - b:%f, g:%f, r:%f", goal->border_color.b,
                 goal->border_color.g, goal->border_color.r);
        zone->color =
            cv::Scalar(goal->border_color.b * 255, goal->border_color.g * 255,
                       goal->border_color.r * 255);
      }
    }
  } else {
    controller->addStaticBorder(
        goal->request_id, goal->zone, goal->position_row, goal->position_col,
        goal->border, goal->border_topic, goal->border_color, goal->is_filled,
        goal->thickness, goal->lifetime, goal->track_violations);
  }

  /*
  if (goal->border.polygon.points.size() > 1) {
  } else {
    controller->addDynamicBorder(goal->request_id, goal->zone,
                                 goal->border_topic, goal->border_color,
                                 goal->is_filled, goal->thickness,
                                 goal->lifetime, goal->track_violations);
  }*/
  sendFeedBackBorder();
  if (as_border.isPreemptRequested() || !ros::ok() || !success) {
    ROS_INFO("%s: Preempted", action_name_border_.c_str());
    // set the action state to preempted
    as_border.setPreempted();
    success = false;
  }
  if (success) {
    ROS_INFO("%s: Succeeded", action_name_border_.c_str());
    // set the action state to succeeded
    sendResultBorder();
  }
}

// send feedback
void SafetyBorderServer::sendFeedBackBorder() {
  feedback_border_.feedback.displayed_request_ids.clear();
  as_border.publishFeedback(feedback_border_.feedback);
}
// send result
void SafetyBorderServer::sendResultBorder() {
  result_border_.displayed_request_ids.clear();
  as_border.setSucceeded(result_border_);
}
