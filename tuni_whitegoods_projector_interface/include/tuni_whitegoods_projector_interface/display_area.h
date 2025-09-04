#ifndef DisplayArea_H
#define DisplayArea_H

#include <geometry_msgs/Point.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tuni_whitegoods_projector_interface/button.h"
#include "tuni_whitegoods_projector_interface/hand.h"
#include "tuni_whitegoods_projector_interface/static_border.h"

struct BorderLayout {
  int rows = 0;
  int cols = 0;
  float sf_factor;
  bool adjacent;
  std_msgs::ColorRGBA status_booked;
  std_msgs::ColorRGBA status_free;
  std_msgs::ColorRGBA status_operator;

  BorderLayout() {
    // Default color: Red for booked
    status_booked.r = 1.0f;
    status_booked.g = 0.0f;
    status_booked.b = 0.0f;
    status_booked.a = 1.0f;

    // Default color: Green for free
    status_free.r = 0.0f;
    status_free.g = 1.0f;
    status_free.b = 0.0f;
    status_free.a = 1.0f;

    // Default color: Blue for operator
    status_operator.r = 0.0f;
    status_operator.g = 0.0f;
    status_operator.b = 1.0f;
    status_operator.a = 1.0f;
  }

  BorderLayout(int r, int c, float sf, bool adj,
               const std_msgs::ColorRGBA& booked,
               const std_msgs::ColorRGBA& free,
               const std_msgs::ColorRGBA& operator_color)
      : rows(r),
        cols(c),
        sf_factor(sf),
        adjacent(adj),
        status_booked(booked),
        status_free(free),
        status_operator(operator_color) {}
};

class DisplayArea {
 private:
  std::vector<std::shared_ptr<StaticBorder>> borders_;
  std::vector<std::shared_ptr<Button>> buttons_;

  ros::Publisher pub_border_violation;
  ros::Publisher pub_button_event;

  ros::NodeHandle* nh_;
  double margin;
  int inner_margin;
  int rect_width;
  int rect_height;
  ros::Time last_detection_time;
  std::vector<cv::Point> left_side_points;
  std::vector<cv::Point> right_side_points;
  std::vector<cv::Point> top_side_points;
  std::vector<cv::Point> bottom_side_points;

 public:
  double projector_rotation_angle;
  double rotation_angle;
  std::string name;
  std::string last_button_pressed;
  bool filling;
  std::string instructions;
  cv::Scalar color;
  cv::Rect display_rect;
  std::vector<geometry_msgs::Point> robot_frame_area;
  std::vector<geometry_msgs::Point> camera_frame_area;
  std::vector<cv::Point> projector_frame_area, inner_projector_frame_area;
  DisplayArea(ros::NodeHandle* nh, std::string name, int projector_id);
  void addBorder(std::shared_ptr<StaticBorder> sb);
  void addButton(std::shared_ptr<Button> btn);
  bool change_button_color(std::string resource_id,
                           std_msgs::ColorRGBA button_color);
  void fetchButtons(std::vector<std::shared_ptr<Button>>& buttons);
  void fetchBorders(std::vector<std::shared_ptr<StaticBorder>>& borders);
  bool checkForInteractions(std::shared_ptr<Hand> hand);
  void resetInteractions();
  void create_border_layout(int rows, int cols, float sf_factor, bool adjacent,
                            std_msgs::ColorRGBA status_booked,
                            std_msgs::ColorRGBA status_free,
                            std_msgs::ColorRGBA status_operator);
  bool robot_book_border(std::string id);
  bool robot_release_border(std::string id, int status);
  bool operator_book_border(std::string id);
  bool operator_release_border(std::string id, int status);
  void setRobotFrame(std::vector<geometry_msgs::Point> robot_frame_points);
  void setCameraFrame(std::vector<geometry_msgs::Point> camera_frame);
  void setProjectorFrame(std::vector<cv::Point> projector_frame);
  geometry_msgs::Pose compute_absolute_world_position(
      geometry_msgs::Pose center);
  cv::Point2f getProjectionCenter();
  BorderLayout border_layout;
  bool containsBorder(std::string border_id);
  void compute_border_dimensions(int rows, int columns);
  std::vector<cv::Point> interpolate(const cv::Point& p1, const cv::Point& p2,
                                     int num_points);
  std::vector<cv::Point> generate_border(int row, int column);
  int projector_id_;
};

#endif