#include "tuni_whitegoods_view/robot_view.h"

#include <visualization_msgs/Marker.h>

using namespace std;

RobotView::RobotView(ros::NodeHandle* nh) {
  vis_pub =
      nh->advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_counter = 1;
  hand_color.r = 1.0;
  hand_color.g = 1.0;
  hand_color.b = 1.0;
  hand_color.a = 1.0;
  ROS_INFO("RobotView running");
}

void RobotView::init(std::vector<std::shared_ptr<DisplayArea>> zones) {
  for (auto& zone : zones) {
    ROS_INFO("Robot view -> add (%s)", zone->name.c_str());
    createRvizMarker(zone->robot_frame_area, hand_color, zone->name);
    ros::Duration(1.0).sleep();
  }
}

void RobotView::updateButtons(
    const std::vector<std::shared_ptr<Button>>& buttons,
    std::shared_ptr<cv::Mat> layer) {
  for (auto& button : buttons) {
    std::vector<geometry_msgs::Point> points(1, button->center.position);
    createRvizMarker(points, button->ros_btn_color, button->get_name());
  }
}

void RobotView::updateBorders(
    const std::vector<std::shared_ptr<StaticBorder>>& borders,
    std::shared_ptr<cv::Mat> layer) {
  for (auto& border : borders) {
    std::vector<geometry_msgs::Point> points;

    points.push_back(border->topLeftCornerPt);
    points.push_back(border->topRightCornerPt);
    points.push_back(border->bottomRightCornerPt);
    points.push_back(border->bottomLeftCornerPt);

    createRvizMarker(points, border->border_color, "border");
  }
}

void RobotView::updateHands(const std::vector<std::shared_ptr<Hand>>& hands) {
  for (auto& hand : hands) {
    std::vector<geometry_msgs::Point> points(1, hand->robot_frame_position);
    createRvizMarker(points, hand_color, "hand", 0);
  }
}

void RobotView::updateDisplayAreas(
    const std::vector<std::shared_ptr<DisplayArea>>& zones) {}

void RobotView::createRvizMarker(std::vector<geometry_msgs::Point> points,
                                 std_msgs::ColorRGBA color,
                                 std::string description, int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base";
  marker.header.stamp = ros::Time::now();
  marker.id = (id == 0) ? id : marker_counter;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.01;
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = 1.0;
  if (points.size() == 1) {
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position = points[0];
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
  } else {
    marker.type = visualization_msgs::Marker::LINE_STRIP;

    for (auto& point : points) {
      marker.points.push_back(point);
    }
    marker.points.push_back(points[0]);
  }

  vis_pub.publish(marker);

  ros::Duration(1.0).sleep();
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = "base";
  text_marker.header.stamp = ros::Time::now();
  text_marker.id = (id == 0) ? 10000 : marker_counter * 10;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.position.x = points[0].x + 0.1;
  text_marker.pose.position.y = points[0].y + 0.1;
  text_marker.pose.position.z = points[0].z + 0.1;
  text_marker.scale.z = 0.2;
  text_marker.color.r = 1.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 1.0f;
  text_marker.color.a = 1.0;
  text_marker.text = description;
  vis_pub.publish(text_marker);

  marker_counter += 1;
}
