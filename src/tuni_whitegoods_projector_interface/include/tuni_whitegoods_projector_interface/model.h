#ifndef ProjectorInterfaceModel_H
#define ProjectorInterfaceModel_H

#include "tuni_whitegoods_projector_interface/display_area.h"
#include "tuni_whitegoods_projector_interface/hand.h"
#include "tuni_whitegoods_projector_interface/button.h"
#include "tuni_whitegoods_projector_interface/static_border.h"


#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

class ProjectorInterfaceModel {
private:
  ros::NodeHandle *nh_;
  std::vector<std::shared_ptr<DisplayArea>> zones;

  std::shared_ptr<Hand> left_hand, right_hand;

  ros::Publisher pub_change_notification;

  ros::ServiceClient client_world_coordinates, client_3D_to_pixel,
      client_pixel_to_3D, client_projector_point,
      client_reverse_projector_point;

public:
  ProjectorInterfaceModel(ros::NodeHandle *nh);
  void add_zone(std::string name);
  void notify();
  void updateHandPose(std::string name, geometry_msgs::Point position);
  std::vector<std::shared_ptr<Button>> getButtons();
  std::vector<std::shared_ptr<StaticBorder>> getBorders();
  std::vector<std::shared_ptr<Hand>> getHands();
};
#endif