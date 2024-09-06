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
  ROS_INFO("Waiting for transform_world_coordinates_frame service to be available...");
  client_world_coordinates.waitForExistence();
  ROS_INFO("transform_world_coordinates_frame service is now available.");

  client_3D_to_pixel =
      nh_->serviceClient<tuni_whitegoods_msgs::Transform3DToPixel>(
          "transform_3D_to_pixel");
  ROS_INFO("Waiting for transform_3D_to_pixel service to be available...");
  client_3D_to_pixel.waitForExistence();
  ROS_INFO("transform_3D_to_pixel service is now available.");

  client_pixel_to_3D =
      nh_->serviceClient<tuni_whitegoods_msgs::TransformPixelTo3D>(
          "transform_pixel_to_3D");
  ROS_INFO("Waiting for transform_pixel_to_3D service to be available...");
  client_pixel_to_3D.waitForExistence();
  ROS_INFO("transform_pixel_to_3D service is now available.");

  client_projector_point =
      nh_->serviceClient<tuni_whitegoods_msgs::TransformPixelToProjection>(
          "transform_point_to_project");
  ROS_INFO("Waiting for transform_point_to_project service to be available...");
  client_projector_point.waitForExistence();
  ROS_INFO("transform_point_to_project service is now available.");

  client_reverse_projector_point =
      nh_->serviceClient<tuni_whitegoods_msgs::TransformPixelToProjection>(
          "reverse_transform_point_to_project");
  ROS_INFO("Waiting for reverse_transform_point_to_project service to be available...");
  client_reverse_projector_point.waitForExistence();
  ROS_INFO("reverse_transform_point_to_project service is now available.");

  left_hand = std::make_shared<Hand>("left");
  right_hand = std::make_shared<Hand>("right");

  pub_change_notification =
      nh->advertise<std_msgs::Empty>("/odin/internal/model_changed", 1);

  // Create a timer 
  interaction_timer_ = nh->createTimer(ros::Duration(1.0), &ProjectorInterfaceModel::reset_interactions, this);

  hands_detected = false;
}

void ProjectorInterfaceModel::create_border_layout(
    int rows, int cols, float sf_factor, bool adjacent,
    std_msgs::ColorRGBA status_booked, std_msgs::ColorRGBA status_free,
    std_msgs::ColorRGBA status_operator){
  for (auto &zone : zones) {
    if(zone->name == "shelf"){
      zone->create_border_layout(rows, cols, sf_factor, adjacent, status_booked, status_free, status_operator);
    }
    break;
  }
}

void ProjectorInterfaceModel::reset_interactions(const ros::TimerEvent&) {
  if (!hands_detected)
  {
    if (!action_triggered) {
      // Check for interaction
      for (auto &zone : zones) {
        zone->resetInteractions();
      }
      // Notify controller
      notify();
      action_triggered = true;
    }
  } else {
    // Reset the variable for the next check
    hands_detected = false;
  }  
}

void ProjectorInterfaceModel::add_zone(std::string name) {
  zones.push_back(std::make_shared<DisplayArea>(nh_, name));
}

void ProjectorInterfaceModel::addButton(std::string request_id, std::string zone, std::string name, std::string text, 
                                        std_msgs::ColorRGBA button_color, std_msgs::ColorRGBA text_color, 
                                        geometry_msgs::Pose center, float radius){
    for (auto &z : zones) {
      if (z->name == zone){
        geometry_msgs::Pose modified_center;
        modified_center.position.x = center.position.x +0.1;
        modified_center.position.y = (request_id == "goButton") ? center.position.y + 0.55 : center.position.y + 0.5;

        modified_center.position.z = -0.40;
        float modified_radius = radius/2.5;
        std::shared_ptr<Button> btn = std::make_shared<Button>(request_id, name, text, 
                                                              button_color, text_color, 
                                                              modified_center, modified_radius);
        geometry_msgs::PoseStamped in_point_stamped_center;
        tuni_whitegoods_msgs::TransformRobotCameraCoordinates srv_center;
        tuni_whitegoods_msgs::Transform3DToPixel srv_3D_to_pixel;

        in_point_stamped_center.header.frame_id = "base";
        in_point_stamped_center.header.stamp = ros::Time(0);
        in_point_stamped_center.pose = modified_center;

        srv_center.request.in_point_stamped = in_point_stamped_center;
        srv_center.request.target_frame = "rgb_camera_link";

        if (!client_world_coordinates.call(srv_center)) {
          ROS_ERROR("Failed to call service");
        } 
        
        // Project to 2D image coordinates
        srv_3D_to_pixel.request.x = srv_center.response.out_point_stamped.pose.position.x;
        srv_3D_to_pixel.request.y = srv_center.response.out_point_stamped.pose.position.y;
        srv_3D_to_pixel.request.z = srv_center.response.out_point_stamped.pose.position.z;
        if (!client_3D_to_pixel.call(srv_3D_to_pixel)) {
          ROS_ERROR("Failed to call service");
        } 

        btn->center_cam_point.x = srv_3D_to_pixel.response.u;
        btn->center_cam_point.y = srv_3D_to_pixel.response.v;

        z->addButton(btn);
        break;
      }
    }
    notify();  // Notify controller
}

void ProjectorInterfaceModel::change_button_color(std::string resource_id, std_msgs::ColorRGBA button_color){
    for (auto &z : zones) {
      if (z->change_button_color(resource_id, button_color)){
        notify();  // Notify controller
        break;
      }
    } 
}

void ProjectorInterfaceModel::addBorder(std::string r_id, std::string z, int pos_row, int pos_col,
                                         geometry_msgs::PolygonStamped bord, std::string b_topic,
                                         std_msgs::ColorRGBA b_color, bool filling, int thic,
                                         ros::Duration life, bool track) {
  for (auto &zone : zones) {
    if (zone->name == z){
      std::shared_ptr<StaticBorder> sb = std::make_shared<StaticBorder>(r_id, pos_row, pos_col,
                                                                        bord, b_topic,
                                                                        b_color, filling, thic,
                                                                        life, track);
      // Also compute camera coordinates for detections
      // TOP LEFT
      // Convert to 3D camera coordinates frame
      geometry_msgs::PoseStamped in_point_stamped_top_left;
      tuni_whitegoods_msgs::TransformRobotCameraCoordinates srv_top_left, srv_bottom_right;
      tuni_whitegoods_msgs::Transform3DToPixel srv_3D_to_pixel;

      in_point_stamped_top_left.header.frame_id = "base";
      in_point_stamped_top_left.header.stamp = ros::Time(0);
      in_point_stamped_top_left.pose.position.x = sb->topLeftCornerPt.x;
      in_point_stamped_top_left.pose.position.y = sb->topLeftCornerPt.y;
      in_point_stamped_top_left.pose.position.z = sb->topLeftCornerPt.z;
      

      srv_top_left.request.in_point_stamped = in_point_stamped_top_left;
      srv_top_left.request.target_frame = "rgb_camera_link";

      if (!client_world_coordinates.call(srv_top_left)) {
        ROS_ERROR("Failed to call service");
      } 
      
      // Project to 2D image coordinates
      srv_3D_to_pixel.request.x = srv_top_left.response.out_point_stamped.pose.position.x;
      srv_3D_to_pixel.request.y = srv_top_left.response.out_point_stamped.pose.position.y;
      srv_3D_to_pixel.request.z = srv_top_left.response.out_point_stamped.pose.position.z;
      if (!client_3D_to_pixel.call(srv_3D_to_pixel)) {
        ROS_ERROR("Failed to call service");
      } 

      sb->top_left_cam_point.x = srv_3D_to_pixel.response.u;
      sb->top_left_cam_point.y = srv_3D_to_pixel.response.v;

      // BOTTOM RIGHT 
      geometry_msgs::PoseStamped in_point_stamped_bottom_right;
      in_point_stamped_bottom_right.header.frame_id = "base";
      in_point_stamped_bottom_right.header.stamp = ros::Time(0);
      in_point_stamped_bottom_right.pose.position.x = sb->bottomRightCornerPt.x;
      in_point_stamped_bottom_right.pose.position.y = sb->bottomRightCornerPt.y;
      in_point_stamped_bottom_right.pose.position.z = sb->bottomRightCornerPt.z;


      srv_bottom_right.request.in_point_stamped = in_point_stamped_bottom_right;
      srv_bottom_right.request.target_frame = "rgb_camera_link";
      if (!client_world_coordinates.call(srv_bottom_right)) {
        ROS_ERROR("Failed to call service");
      } 

      srv_3D_to_pixel.request.x = srv_bottom_right.response.out_point_stamped.pose.position.x;
      srv_3D_to_pixel.request.y = srv_bottom_right.response.out_point_stamped.pose.position.y;
      srv_3D_to_pixel.request.z = srv_bottom_right.response.out_point_stamped.pose.position.z;
      if (!client_3D_to_pixel.call(srv_3D_to_pixel)) {
        ROS_ERROR("Failed to call service");
      } 

      sb->bottom_right_cam_point.x = srv_3D_to_pixel.response.u;
      sb->bottom_right_cam_point.y = srv_3D_to_pixel.response.v;

      zone->addBorder(sb);

      break;
    }
  }
  // Notify controller
  notify();
}

void ProjectorInterfaceModel::robot_book_border(std::string id) {
  for (auto &z : zones) {
    if (z->robot_book_border(id)){
      notify();  // Notify controller
      break;
    }
  } 
}

void ProjectorInterfaceModel::robot_release_border(std::string id, int status) {
  for (auto &z : zones) {
    if (z->robot_release_border(id, status)){
      notify();  // Notify controller
      break;
    }
  } 
}

void ProjectorInterfaceModel::operator_book_border(std::string id) {
  for (auto &z : zones) {
    if (z->operator_book_border(id)){
      notify();  // Notify controller
      break;
    }
  } 
}

void ProjectorInterfaceModel::operator_release_border(std::string id, int status) {
  for (auto &z : zones) {
    if (z->operator_release_border(id, status)){
      notify();  // Notify controller
      break;
    }
  } 
}

void ProjectorInterfaceModel::notify() {
  pub_change_notification.publish(std_msgs::Empty());
}

void ProjectorInterfaceModel::updateHandPose(std::string name,
                                               geometry_msgs::Point position) {
  hands_detected = true;
  action_triggered = false; 
  // Project pixel position to 3D
  tuni_whitegoods_msgs::TransformPixelTo3D srv_pixel_to_3D;
  srv_pixel_to_3D.request.u = position.x;
  srv_pixel_to_3D.request.v = position.y;
  srv_pixel_to_3D.request.depth = position.z;
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