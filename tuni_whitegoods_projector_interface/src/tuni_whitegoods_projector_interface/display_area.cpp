#include "tuni_whitegoods_projector_interface/display_area.h"

#include <integration/SafetyBorderViolation.h>
#include <integration/VirtualButtonEventArray.h>

#include <cmath>

DisplayArea::DisplayArea(ros::NodeHandle *nh, std::string name)
    : nh_(nh), name(name), margin(100), inner_margin(50) {
  pub_border_violation = nh->advertise<integration::SafetyBorderViolation>(
      "/execution/projector_interface/integration/topics/"
      "safety_border_violation",
      1);
  pub_button_event = nh_->advertise<integration::VirtualButtonEventArray>(
      "/execution/projector_interface/integration/topics/"
      "virtual_button_event_array",
      1);
}

void DisplayArea::create_border_layout(int rows, int cols, float sf_factor,
                                       bool adjacent,
                                       std_msgs::ColorRGBA status_booked,
                                       std_msgs::ColorRGBA status_free,
                                       std_msgs::ColorRGBA status_operator) {
  border_layout = {rows,          cols,        sf_factor,      adjacent,
                   status_booked, status_free, status_operator};
  compute_border_dimensions(rows, cols);
}

void DisplayArea::compute_border_dimensions(int rows, int columns) {
  ROS_INFO("table projector_frame_area top left: x = %d, y = %d",
           projector_frame_area[0].x, projector_frame_area[0].y);
  ROS_INFO("table projector_frame_area top left: x = %d, y = %d",
           projector_frame_area[1].x, projector_frame_area[1].y);
  ROS_INFO("table projector_frame_area top left: x = %d, y = %d",
           projector_frame_area[2].x, projector_frame_area[2].y);
  ROS_INFO("table projector_frame_area top left: x = %d, y = %d",
           projector_frame_area[3].x, projector_frame_area[3].y);

  // Calculate rectangle width and height with inner margins
  int width_with_margin = static_cast<int>(
      cv::norm(projector_frame_area[1] - projector_frame_area[0]) - 2 * margin);
  int height_with_margin = static_cast<int>(
      cv::norm(projector_frame_area[3] - projector_frame_area[0]) - 2 * margin);

  rect_width = (width_with_margin - (columns - 1) * inner_margin) / columns;
  rect_height = (height_with_margin - (rows - 1) * inner_margin) / rows;
  ROS_INFO("rect_width: %d", rect_width);
  ROS_INFO("rect_height: %d", rect_height);

  cv::Point inner_top_left;
  cv::Point inner_top_right;
  cv::Point inner_bottom_left;
  cv::Point inner_bottom_right;

  if (projector_frame_area[0].x < projector_frame_area[1].x) {
    ROS_INFO("left > right");
    inner_top_left.x = projector_frame_area[0].x + margin;
    inner_top_right.x = projector_frame_area[1].x - margin;
  } else {
    ROS_INFO("right > left");
    inner_top_left.x = projector_frame_area[1].x + margin;
    inner_top_right.x = projector_frame_area[0].x - margin;
  }

  inner_bottom_right.x = inner_top_right.x;
  inner_bottom_left.x = inner_top_left.x;

  if (projector_frame_area[0].y < projector_frame_area[3].y) {
    ROS_INFO("top > down");
    inner_top_left.y = projector_frame_area[3].y - margin;
    inner_bottom_left.y = projector_frame_area[3].y + margin;
  } else {
    ROS_INFO("bottom > up");
    inner_top_left.y = projector_frame_area[3].y + margin;
    inner_bottom_left.y = projector_frame_area[0].y - margin;
  }

  inner_top_right.y = inner_top_left.y;
  inner_bottom_right.y = inner_bottom_left.y;

  inner_projector_frame_area = {inner_top_left, inner_top_right,
                                inner_bottom_right, inner_bottom_left};

  ROS_INFO("table inner projector_frame_area top left: x = %d, y = %d",
           inner_projector_frame_area[0].x, inner_projector_frame_area[0].y);
  ROS_INFO("table inner projector_frame_area top left: x = %d, y = %d",
           inner_projector_frame_area[1].x, inner_projector_frame_area[1].y);
  ROS_INFO("table inner projector_frame_area top left: x = %d, y = %d",
           inner_projector_frame_area[2].x, inner_projector_frame_area[2].y);
  ROS_INFO("table inner projector_frame_area top left: x = %d, y = %d",
           inner_projector_frame_area[3].x, inner_projector_frame_area[3].y);

  left_side_points =
      interpolate(inner_projector_frame_area[0], inner_projector_frame_area[3],
                  border_layout.rows);

  right_side_points =
      interpolate(inner_projector_frame_area[1], inner_projector_frame_area[2],
                  border_layout.rows);

  top_side_points =
      interpolate(inner_projector_frame_area[0], inner_projector_frame_area[1],
                  border_layout.cols);

  bottom_side_points =
      interpolate(inner_projector_frame_area[3], inner_projector_frame_area[2],
                  border_layout.cols);
}

std::vector<cv::Point> DisplayArea::interpolate(const cv::Point &p1,
                                                const cv::Point &p2,
                                                int num_points) {
  std::vector<cv::Point> points;
  for (int i = 0; i < num_points + 1; ++i) {
    double t = static_cast<double>(i) / (num_points);
    int x = static_cast<int>(p1.x + (p2.x - p1.x) * t);
    int y = static_cast<int>(p1.y + (p2.y - p1.y) * t);
    points.push_back(cv::Point(x, y));
  }
  return points;
}

std::vector<cv::Point> DisplayArea::generate_border(int row, int column) {
  cv::Point top_left;
  cv::Point top_right;
  cv::Point bottom_right;

  top_left.x = top_side_points[column - 1].x + inner_margin / 2;
  top_right.x = top_left.x + rect_width;
  bottom_right.x = top_right.x;

  top_left.y = left_side_points[row - 1].y + inner_margin / 2;
  top_right.y = top_left.y;
  bottom_right.y = top_right.y + rect_height;

  cv::Point bottom_left(top_left.x, bottom_right.y);

  std::vector<cv::Point> result = {top_left, top_right, bottom_right,
                                   bottom_left};
  return result;
}

bool DisplayArea::checkForInteractions(
    const std::string &name, const geometry_msgs::Point &hand_position) {
  bool result = false;
  float distance;
  cv::Point cv_hand_position(static_cast<int>(hand_position.x),
                             static_cast<int>(hand_position.y));

  for (auto &border : borders_) {
    if (border->robot_booked || border->operator_booked) {
      if (border->checkForInteractions(name, cv_hand_position)) {
        result = true;
        // OpenFlow signal
        integration::SafetyBorderViolation msg_border;
        geometry_msgs::PolygonStamped initial_border;
        geometry_msgs::Pose target_location;

        msg_border.header.frame_id = "base";
        msg_border.request_id = border->getId();

        // initial_border.polygon.points.push_back();
        // initial_border.polygon.points.push_back();
        // initial_border.polygon.points.push_back();
        // initial_border.polygon.points.push_back();

        // target_location.position = ;

        // msg_border.initial_border = initial_border;
        // msg_border.target_location = target_location;
        pub_border_violation.publish(msg_border);
      }
    }
  }

  for (auto &button : buttons_) {
    integration::VirtualButtonEventArray events;
    if (button->checkForInteractions(name, cv_hand_position)) {
      if (!button->isAlreadyPressed()) {
        result = true;
        button->setAlreadyPressed(true);
        integration::VirtualButtonEvent msg_event;
        msg_event.virtual_button_id = button->getId();
        msg_event.event_type = msg_event.PRESSED;
        events.virtual_button_events.push_back(msg_event);
        pub_button_event.publish(events);
      }
    } else {
      if (button->isAlreadyPressed()) {
        integration::VirtualButtonEvent msg_event;
        msg_event.virtual_button_id = button->getId();
        msg_event.event_type = msg_event.RELEASED;
        events.virtual_button_events.push_back(msg_event);
        pub_button_event.publish(events);

        button->setAlreadyPressed(false);
      }
    }
  }

  return result;
}

void DisplayArea::resetInteractions() {
  for (auto &border : borders_) {
    border->resetInteractions();
  }

  for (auto &button : buttons_) {
    button->resetInteractions();
  }
}

void DisplayArea::addButton(std::shared_ptr<Button> btn) {
  buttons_.push_back(btn);
}

bool DisplayArea::change_button_color(std::string resource_id,
                                      std_msgs::ColorRGBA button_color) {
  bool result = false;
  for (auto &button : buttons_) {
    if (button->get_name() == resource_id) {
      button->set_button_color(button_color);
      result = true;
      break;
    }
  }
  return result;
}

geometry_msgs::Pose DisplayArea::compute_absolute_world_position(
    geometry_msgs::Pose center) {
  geometry_msgs::Pose result;
  result.position.x =
      camera_frame_area[0].x +
      (camera_frame_area[1].x - camera_frame_area[0].x) * center.position.x;
  result.position.y =
      camera_frame_area[0].y +
      (camera_frame_area[3].y - camera_frame_area[0].y) * center.position.y;
  result.position.z = center.position.z;
  return result;
}

bool DisplayArea::robot_book_border(std::string id) {
  bool result = false;
  for (auto &border : borders_) {
    if (border->getId() == id) {
      border->robot_book(border_layout.status_booked);
      if (border_layout.adjacent) {
        for (auto &adj_border : borders_) {
          if (border->isAdjacent(adj_border)) {
            adj_border->robot_book(border_layout.status_booked);
          }
        }
      }
      result = true;
      break;
    }
  }
  return result;
}

bool DisplayArea::robot_release_border(std::string id, int status) {
  bool result = false;
  for (auto &border : borders_) {
    if (border->getId() == id) {
      border->release(border_layout.status_free);
      border->resetInteractions();
      if (status == -1) {
        for (auto &adj_border : borders_) {
          if (border->isAdjacent(adj_border)) {
            adj_border->release(border_layout.status_free);
            adj_border->resetInteractions();
          }
        }
      }
      result = true;
      break;
    }
  }
  return result;
}

bool DisplayArea::operator_book_border(std::string id) {
  bool result = false;
  for (auto &border : borders_) {
    if (border->getId() == id) {
      border->operator_book(border_layout.status_operator);
      result = true;
      break;
    }
  }
  return result;
}

bool DisplayArea::operator_release_border(std::string id, int status) {
  bool result = false;
  for (auto &border : borders_) {
    if (border->getId() == id) {
      border->release(border_layout.status_free);
      border->resetInteractions();
      if (status == -1) {
        for (auto &adj_border : borders_) {
          if (border->isAdjacent(adj_border)) {
            adj_border->release(border_layout.status_free);
            adj_border->resetInteractions();
          }
        }
      }
      result = true;
      break;
    }
  }
  return result;
}

void DisplayArea::fetchButtons(std::vector<std::shared_ptr<Button>> &buttons) {
  for (auto button : buttons_) {
    buttons.push_back(button);
  }
}

void DisplayArea::addBorder(std::shared_ptr<StaticBorder> sb) {
  borders_.push_back(sb);
}

void DisplayArea::fetchBorders(
    std::vector<std::shared_ptr<StaticBorder>> &borders) {
  for (auto border : borders_) {
    borders.push_back(border);
  }
}

void DisplayArea::setRobotFrame(
    std::vector<geometry_msgs::Point> robot_frame_points) {
  robot_frame_area = robot_frame_points;
}

void DisplayArea::setCameraFrame(
    std::vector<geometry_msgs::Point> camera_frame) {
  camera_frame_area = camera_frame;
}

void DisplayArea::setProjectorFrame(std::vector<cv::Point> projector_frame) {
  projector_frame_area = projector_frame;
}
