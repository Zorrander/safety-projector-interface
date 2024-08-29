#include "tuni_whitegoods_projector_interface/model.h"

#include <std_msgs/Empty.h>

#include <geometry_msgs/PoseStamped.h>

#include "tuni_whitegoods_msgs/Transform3DToPixel.h"
#include "tuni_whitegoods_msgs/TransformPixelTo3D.h"
#include "tuni_whitegoods_msgs/TransformPixelToProjection.h"
#include "tuni_whitegoods_msgs/TransformRobotCameraCoordinates.h"

ProjectorInterfaceModel::ProjectorInterfaceModel(ros::NodeHandle *nh)
    : nh_(nh) {
  // Start clients for transform
  client_world_coordinates =
      nh_->serviceClient<tuni_whitegoods_msgs::TransformRobotCameraCoordinates>(
          "transform_world_coordinates_frame");
  client_3D_to_pixel =
      nh_->serviceClient<tuni_whitegoods_msgs::Transform3DToPixel>(
          "transform_3D_to_pixel");
  client_pixel_to_3D =
      nh_->serviceClient<tuni_whitegoods_msgs::TransformPixelTo3D>(
          "transform_pixel_to_3D");
  client_projector_point =
      nh_->serviceClient<tuni_whitegoods_msgs::TransformPixelToProjection>(
          "transform_point_to_project");
  client_reverse_projector_point =
      nh_->serviceClient<tuni_whitegoods_msgs::TransformPixelToProjection>(
          "reverse_transform_point_to_project");

  left_hand = std::make_shared<Hand>("left");
  right_hand = std::make_shared<Hand>("right");

  pub_change_notification =
      nh->advertise<std_msgs::Empty>("/odin/internal/model_changed", 1);
}

void ProjectorInterfaceModel::add_zone(std::string name) {
  zones.push_back(std::make_shared<DisplayArea>(name));
}

void ProjectorInterfaceModel::notify() {
  pub_change_notification.publish(std_msgs::Empty());
}

void ProjectorInterfaceModel::updateHandPose(std::string name,
                                               geometry_msgs::Point position) {
  // Project pixel position to 3D
  tuni_whitegoods_msgs::TransformPixelTo3D srv_pixel_to_3D;
  srv_pixel_to_3D.request.u = position.x;
  srv_pixel_to_3D.request.v = position.y;
  client_pixel_to_3D.call(srv_pixel_to_3D);

  geometry_msgs::Point projected;
  projected.x = srv_pixel_to_3D.response.x;
  projected.y = srv_pixel_to_3D.response.y;
  projected.z = srv_pixel_to_3D.response.z;
  // Transform to robot coordinates frame
  geometry_msgs::PoseStamped in_point_stamped;
  in_point_stamped.header.frame_id = "rgb_camera_link";
  in_point_stamped.header.stamp = ros::Time(0);
  in_point_stamped.pose.position = projected;

  tuni_whitegoods_msgs::TransformRobotCameraCoordinates srv;
  srv.request.in_point_stamped = in_point_stamped;
  srv.request.target_frame = "base";
  client_world_coordinates.call(srv);

  if (name == "left") {
    left_hand->set_position(position, projected, srv.response.out_point_stamped.pose.position);
  } else if (name == "right") {
    right_hand->set_position(position, projected, srv.response.out_point_stamped.pose.position);
  }
  // Check for interaction
  for (auto &zone : zones) {
    zone->checkForInteractions(name, position);
  }
  // Notify controller
  notify();
}

std::vector<std::shared_ptr<Button>> ProjectorInterfaceModel::getButtons(){
  std::vector<std::shared_ptr<Button>> buttons;
  for (auto &zone : zones) {
    zone->fetchButtons(buttons);
  }
  return buttons;
}

std::vector<std::shared_ptr<StaticBorder>> ProjectorInterfaceModel::getBorders(){
  std::vector<std::shared_ptr<StaticBorder>> borders;
  for (auto &zone : zones) {
    zone->fetchBorders(borders);
  }
  return borders;
}

std::vector<std::shared_ptr<Hand>> ProjectorInterfaceModel::getHands(){
  std::vector<std::shared_ptr<Hand>> hands;
  hands.push_back(left_hand);
  hands.push_back(right_hand);
  return hands;
}