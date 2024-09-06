#ifndef ProjectorInterfaceController_H
#define ProjectorInterfaceController_H

#include <memory>
#include <ros/ros.h>
#include <vector>

#include <tuni_whitegoods_view/view.h>

#include "tuni_whitegoods_projector_interface/model.h"

#include "tuni_whitegoods_perception/object_detector.h"

#include "tuni_whitegoods_msgs/DynamicArea.h"
#include "tuni_whitegoods_msgs/HandsState.h"

#include <geometry_msgs/PolygonStamped.h>
#include <integration/ListStaticBordersStatus.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Empty.h>


class ProjectorInterfaceController {
protected:
  ros::NodeHandle *nh_;

  std::vector<std::shared_ptr<View>> views;

  std::shared_ptr<View> projector_view, camera_view, robot_view;

  std::shared_ptr<ObjectDetector> detector;

  std::unique_ptr<ProjectorInterfaceModel> model_;

  ros::Subscriber hand_pose_sub, moving_table_pose_sub, model_update_sub, rgb_sub;

  ros::Publisher pub_button_event;

  ros::ServiceServer service_borders;
  cv::Mat cv_rgb;

public:
  ProjectorInterfaceController(ros::NodeHandle *nh);
  void notify();
  void createBorderLayout(int rows, int cols, float sf_factor, bool adjacent,
                          std_msgs::ColorRGBA status_booked,
                          std_msgs::ColorRGBA status_free,
                          std_msgs::ColorRGBA status_operator);

  void movingTableTrackerCallback(const tuni_whitegoods_msgs::DynamicArea &msg);
  void handTrackerCallback(const tuni_whitegoods_msgs::HandsState &msg);
  void addButton(std::string request_id, std::string zone, std::string name, std::string text, 
                 std_msgs::ColorRGBA button_color, std_msgs::ColorRGBA text_color, 
                 geometry_msgs::Pose center, float radius);
  void change_button_color(std::string resource_id, std_msgs::ColorRGBA button_color);
  void addBorder(std::string r_id, std::string z, int pos_row, int pos_col,
                 geometry_msgs::PolygonStamped bord, std::string b_topic,
                 std_msgs::ColorRGBA b_color, bool filling, int thic,
                 ros::Duration life, bool track);
  void robot_book_border(std::string id);
  void operator_book_border(std::string id);
  void robot_release_border(std::string id, int status);
  void operator_release_border(std::string id, int status);
  std::vector<std::string> getAdjacentBorders(int row, int col);
  bool getBordersService(integration::ListStaticBordersStatus::Request &req,
                         integration::ListStaticBordersStatus::Response &res);
  void modelUpdateCallback(const std_msgs::Empty &msg);
  void rgbImageCallback(const sensor_msgs::ImageConstPtr& rgb_msg);
  // void callback_button(const unity_msgs::ElementUI::ConstPtr &msg);
  // void process_button(double center_x, double center_y, const
  // unity_msgs::ElementUI &msg); void callback_button_color(const
  // unity_msgs::ElementUI::ConstPtr &msg);
};
#endif