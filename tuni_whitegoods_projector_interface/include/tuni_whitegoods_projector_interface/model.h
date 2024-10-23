#ifndef ProjectorInterfaceModel_H
#define ProjectorInterfaceModel_H

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include "tuni_whitegoods_msgs/DynamicArea.h"
#include "tuni_whitegoods_projector_interface/button.h"
#include "tuni_whitegoods_projector_interface/display_area.h"
#include "tuni_whitegoods_projector_interface/hand.h"
#include "tuni_whitegoods_projector_interface/static_border.h"

class ProjectorInterfaceModel {
 private:
  ros::NodeHandle* nh_;

  std::shared_ptr<Hand> left_hand, right_hand;

  ros::Publisher pub_change_notification;

  ros::ServiceClient client_world_coordinates, client_3D_to_pixel,
      client_pixel_to_3D, client_projector_point,
      client_reverse_projector_point;

  ros::Timer interaction_timer_;

  bool hands_detected, action_triggered, hand_visualization;

 public:
  ProjectorInterfaceModel(ros::NodeHandle* nh);
  void create_border_layout(int rows, int cols, float sf_factor, bool adjacent,
                            std_msgs::ColorRGBA status_booked,
                            std_msgs::ColorRGBA status_free,
                            std_msgs::ColorRGBA status_operator);
  void add_zone(std::shared_ptr<DisplayArea> display_area,
                std::vector<geometry_msgs::Point> camera_frame);

  void addButton(std::string request_id, std::string zone, std::string name,
                 std::string text, std_msgs::ColorRGBA button_color,
                 std_msgs::ColorRGBA text_color, geometry_msgs::Pose center,
                 float radius);
  void change_button_color(std::string resource_id,
                           std_msgs::ColorRGBA button_color);

  void addStaticBorder(cv::Mat depth_img, std::string r_id, std::string z,
                       int pos_row, int pos_col,
                       geometry_msgs::PolygonStamped bord, std::string b_topic,
                       std_msgs::ColorRGBA b_color, bool filling, int thic,
                       ros::Duration life, bool track);
  void addDynamicBorder(std::string r_id, std::string z, std::string b_topic,
                        std_msgs::ColorRGBA b_color, bool filling, int thic,
                        ros::Duration life, bool track);
  void notify();

  void updateHandPose(const std::string& name,
                      const geometry_msgs::Point& position);

  void updateMovingTable(const tuni_whitegoods_msgs::DynamicArea& moving_table);

  void reset_interactions(const ros::TimerEvent&);

  void robot_book_border(std::string id);
  void robot_release_border(std::string id, int status);

  void operator_book_border(std::string id);
  void operator_release_border(std::string id, int status);

  std::vector<std::shared_ptr<Button>> getButtons();
  std::vector<std::shared_ptr<StaticBorder>> getBorders();
  std::vector<std::shared_ptr<Hand>> getHands();

  std::vector<std::shared_ptr<DisplayArea>> getDisplayAreas();

  cv::Point fromRobot2Pixel(geometry_msgs::Pose pose);
  geometry_msgs::Pose fromPixel2Robot(geometry_msgs::Point pixel);
  cv::Point fromProjector2Camera(cv::Point pixel);
  cv::Point fromCamera2Projector(geometry_msgs::Point pixel);
  std::vector<std::shared_ptr<DisplayArea>> zones;
};
#endif