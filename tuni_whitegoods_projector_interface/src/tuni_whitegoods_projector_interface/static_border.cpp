/*
Class to create StaticBorder
*/
#include "tuni_whitegoods_projector_interface/static_border.h"

// The parameters are defined in OpenFlow
// id : id of the border
// z : zone where to display the border
// pos_row, pos_col : the virtual position of this border in a layout
// bord : the border (top-left, bottom-right) of the border
// b_color : color of the border
// filling : if the border should be filled
// thic : thickness of the border
// life : lifetime of the border. Not implemented here because not really useful
// track : if we track the border (monitor any crossing)
StaticBorder::StaticBorder(ros::NodeHandle* nh, std::string r_id, int pos_row,
                           int pos_col, geometry_msgs::PolygonStamped bord,
                           std::string b_topic, std_msgs::ColorRGBA b_color,
                           bool filling, int thic, ros::Duration life,
                           bool track) {
  ros::param::get("camera_resolution", camera_resolution);
  ros::param::get("shelf_height", shelf_height);

  request_id = r_id;
  position_row = pos_row;
  position_col = pos_col;

  topLeftCornerPt.x = bord.polygon.points[0].x;
  topLeftCornerPt.y = bord.polygon.points[0].y;
  topLeftCornerPt.z = shelf_height;

  topRightCornerPt.x = bord.polygon.points[1].x;
  topRightCornerPt.y = bord.polygon.points[1].y;
  topRightCornerPt.z = shelf_height;

  bottomRightCornerPt.x = bord.polygon.points[2].x;
  bottomRightCornerPt.y = bord.polygon.points[2].y;
  bottomRightCornerPt.z = shelf_height;

  bottomLeftCornerPt.x = bord.polygon.points[3].x;
  bottomLeftCornerPt.y = bord.polygon.points[3].y;
  bottomLeftCornerPt.z = shelf_height;

  border_color = b_color;
  is_filled = filling;
  thickness = thic;
  lifetime = life;
  track_violations = track;
  left_hand_crossed = false;
  right_hand_crossed = false;
  robot_booked = false;
  operator_booked = false;
}

// draw a border
cv::Mat StaticBorder::drawBorder() {
  sf_line_colored =
      cv::Mat::zeros(camera_resolution[1], camera_resolution[0], CV_8UC3);
  cv::rectangle(sf_line_colored, top_left_cam_point, bottom_right_cam_point,
                cv::Scalar(border_color.b * 255, border_color.g * 255,
                           border_color.r * 255),
                thickness, cv::LINE_8);
  return sf_line_colored;
}

// change the color of the border
void StaticBorder::changeBorderColor(std_msgs::ColorRGBA col) {
  border_color.r = col.r;
  border_color.g = col.g;
  border_color.b = col.b;
  border_color.a = col.a;
}

void StaticBorder::changeThickness(int thic) { thickness = thic; }

void StaticBorder::robot_book(std_msgs::ColorRGBA col) {
  robot_booked = true;
  changeBorderColor(col);
}

void StaticBorder::operator_book(std_msgs::ColorRGBA col) {
  operator_booked = true;
  changeBorderColor(col);
}

void StaticBorder::release(std_msgs::ColorRGBA col) {
  robot_booked = false;
  operator_booked = false;
  changeBorderColor(col);
}

// get the diagonal of the border. Used for hand detection if a hand's location
// is less than diagonal*factor then it throws a violation
float StaticBorder::getBorderDiagonal() {
  float dist = sqrt(pow((top_left_cam_point.x - bottom_right_cam_point.x), 2) +
                    pow((top_left_cam_point.y - bottom_right_cam_point.y), 2));
  return dist;
}

// get the center of the border
cv::Point StaticBorder::getCenter() {
  cv::Point p;
  p.x = (top_left_cam_point.x + bottom_right_cam_point.x) / 2;
  p.y = (top_left_cam_point.y + bottom_right_cam_point.y) / 2;

  return p;
}

bool StaticBorder::checkForInteractions(const std::string& name,
                                        const cv::Point& hand_position) {
  bool result = false;
  float distance = cv::norm(hand_position - getCenter());
  bool is_crossed = distance < getBorderDiagonal() * 0.5;
  if (name == "left") {
    left_hand_crossed = is_crossed;
  } else if (name == "right") {
    right_hand_crossed = is_crossed;
  }

  border_violated = (right_hand_crossed || left_hand_crossed);

  if (!border_violated) {
    thickness = 1;
  } else {
    if (robot_booked) {
      result = true;
      thickness = -1;
    } else if (operator_booked) {
      thickness = 3;
    }
  }
  return result;
}

void StaticBorder::resetInteractions() {
  left_hand_crossed = false;
  right_hand_crossed = false;
  border_violated = false;
  thickness = 1;
}

bool StaticBorder::isAdjacent(std::shared_ptr<StaticBorder> sb) {
  bool result = false;
  if (sb->getCol() == position_col + 1 || sb->getCol() == position_col - 1) {
    result = true;
  }
  return result;
}

// return the zone where the border should be displayed
std::string StaticBorder::getZone() const { return zone; }
// return the id of the border
std::string StaticBorder::getId() { return request_id; }
// return if the border should be monitored
bool StaticBorder::getTracking() { return track_violations; }
// get the coloumn location of the border within the virtual layout
int StaticBorder::getCol() { return position_col; }
// get the row location of the border within the virtual layout
int StaticBorder::getRow() { return position_row; }
// get the border color
std_msgs::ColorRGBA StaticBorder::getColor() { return border_color; }
