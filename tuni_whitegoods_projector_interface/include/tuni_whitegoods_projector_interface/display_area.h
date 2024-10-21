#ifndef DisplayArea_H
#define DisplayArea_H

#include <geometry_msgs/Point.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tuni_whitegoods_projector_interface/button.h"
#include "tuni_whitegoods_projector_interface/static_border.h"

struct BorderLayout {
  int rows;
  int cols;
  float sf_factor;
  bool adjacent;
  std_msgs::ColorRGBA status_booked;
  std_msgs::ColorRGBA status_free;
  std_msgs::ColorRGBA status_operator;
};

class DisplayArea {
 private:
  std::vector<geometry_msgs::Point> robot_frame_area;
  cv::Rect camera_frame_area;
  cv::Rect projector_frame_area;

  std::vector<std::shared_ptr<StaticBorder>> borders_;
  std::vector<std::shared_ptr<Button>> buttons_;

  BorderLayout border_layout;

  ros::Publisher pub_border_violation;
  ros::Publisher pub_button_event;

  ros::NodeHandle* nh_;

 public:
  std::string name;

  DisplayArea(ros::NodeHandle* nh, std::string name);
  void addBorder(std::shared_ptr<StaticBorder> sb);
  void addButton(std::shared_ptr<Button> btn);
  bool change_button_color(std::string resource_id,
                           std_msgs::ColorRGBA button_color);
  void fetchButtons(std::vector<std::shared_ptr<Button>>& buttons);
  void fetchBorders(std::vector<std::shared_ptr<StaticBorder>>& borders);
  bool checkForInteractions(const std::string& name,
                            const geometry_msgs::Point& hand_position);
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
  void setCameraFrame(cv::Rect camera_frame);
  void setProjectorFrame(cv::Rect projector_frame);
};

#endif