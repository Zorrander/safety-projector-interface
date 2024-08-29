#ifndef ProjectorInterfaceController_H
#define ProjectorInterfaceController_H

#include <memory>
#include <ros/ros.h>
#include <vector>

#include <tuni_whitegoods_view/view.h>

#include "tuni_whitegoods_projector_interface/model.h"

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

  std::unique_ptr<ProjectorInterfaceModel> model_;

  ros::Subscriber hand_pose_sub, moving_table_pose_sub, model_update_sub;

  ros::Publisher pub_button_event;

  ros::ServiceServer service_borders;

public:
  ProjectorInterfaceController(ros::NodeHandle *nh);
  void notify();
  void createBorderLayout(int rows, int cols, float sf_factor, bool adjacent,
                          std_msgs::ColorRGBA status_booked,
                          std_msgs::ColorRGBA status_free,
                          std_msgs::ColorRGBA status_operator);

  void movingTableTrackerCallback(const tuni_whitegoods_msgs::DynamicArea &msg);
  void handTrackerCallback(const tuni_whitegoods_msgs::HandsState &msg);

  void addBorder(std::string r_id, std::string z, int pos_row, int pos_col,
                 geometry_msgs::PolygonStamped bord, std::string b_topic,
                 std_msgs::ColorRGBA b_color, bool filling, int thic,
                 ros::Duration life, bool track);
  void bookBorderRobot(std::string id);
  void bookBorderOperator(std::string id);
  void releaseRobotBorder(std::string id, int status);
  void releaseOperatorBorder(std::string id, int status);
  std::vector<std::string> getAdjacentBorders(int row, int col);
  bool getBordersService(integration::ListStaticBordersStatus::Request &req,
                         integration::ListStaticBordersStatus::Response &res);
  void modelUpdateCallback(const std_msgs::Empty &msg);
  // void callback_button(const unity_msgs::ElementUI::ConstPtr &msg);
  // void process_button(double center_x, double center_y, const
  // unity_msgs::ElementUI &msg); void callback_button_color(const
  // unity_msgs::ElementUI::ConstPtr &msg);
};
#endif