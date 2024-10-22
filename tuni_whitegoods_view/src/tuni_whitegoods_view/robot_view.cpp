#include "tuni_whitegoods_view/robot_view.h"

#include <visualization_msgs/Marker.h>

using namespace std;

RobotView::RobotView(ros::NodeHandle* nh) {
  vis_pub =
      nh->advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_counter = 1;
  ROS_INFO("RobotView running");
}

void RobotView::init(std::vector<std::shared_ptr<DisplayArea>> zones) {
  ROS_INFO("RobotView init");
}

void RobotView::updateButtons(
    const std::vector<std::shared_ptr<Button>>& buttons) {
  for (auto& button : buttons) {
    std::vector<geometry_msgs::Point> points(1, button->center.position);
    createRvizMarker(points, button->ros_btn_color);
  }
}

void RobotView::updateBorders(
    const std::vector<std::shared_ptr<StaticBorder>>& borders) {
  for (auto& border : borders) {
    std::vector<geometry_msgs::Point> points;

    points.push_back(border->topLeftCornerPt);
    points.push_back(border->topRightCornerPt);
    points.push_back(border->bottomRightCornerPt);
    points.push_back(border->bottomLeftCornerPt);

    createRvizMarker(points, border->border_color);
  }
}

void RobotView::updateHands(const std::vector<std::shared_ptr<Hand>>& hands) {
  for (auto& hand : hands) {
    std::vector<geometry_msgs::Point> points(1, hand->robot_frame_position);
    createRvizMarker(points, hand_color, 0);
  }
}

void RobotView::createRvizMarker(std::vector<geometry_msgs::Point> points,
                                 std_msgs::ColorRGBA color, int id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base";
  marker.header.stamp = ros::Time::now();
  marker.id = (id == 0) ? id : marker_counter;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.01;  // Line width

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
  marker_counter += 1;
}
