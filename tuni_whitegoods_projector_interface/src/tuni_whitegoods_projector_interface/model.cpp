#include "tuni_whitegoods_projector_interface/model.h"

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>

#include <Eigen/Dense>
#include <boost/algorithm/clamp.hpp>

#include "tuni_whitegoods_msgs/Transform3DToPixel.h"
#include "tuni_whitegoods_msgs/TransformPixelTo3D.h"
#include "tuni_whitegoods_msgs/TransformPixelToProjection.h"
#include "tuni_whitegoods_msgs/TransformRobotCameraCoordinates.h"

ProjectorInterfaceModel::ProjectorInterfaceModel(ros::NodeHandle *nh)
    : nh_(nh) {
  client_world_coordinates =
      nh_->serviceClient<tuni_whitegoods_msgs::TransformRobotCameraCoordinates>(
          "transform_world_coordinates_frame");
  ROS_INFO(
      "Waiting for transform_world_coordinates_frame service to be "
      "available...");
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

  client_projector_smart_interface =
      nh_->serviceClient<tuni_whitegoods_msgs::TransformPixelToProjection>(
          "transform_point_to_smart_interface");
  ROS_INFO("Waiting for transform_point_to_project service to be available...");
  client_projector_smart_interface.waitForExistence();
  ROS_INFO("transform_point_to_project service is now available.");

  client_reverse_projector_point =
      nh_->serviceClient<tuni_whitegoods_msgs::TransformPixelToProjection>(
          "reverse_transform_point_to_project");
  ROS_INFO(
      "Waiting for reverse_transform_point_to_project service to be "
      "available...");
  client_reverse_projector_point.waitForExistence();
  ROS_INFO("reverse_transform_point_to_project service is now available.");

  ros::param::get("hand_visualization", hand_visualization);
  ros::param::get("/button_homography", button_homography_array);
  button_homography = cv::Matx33d(button_homography_array.data());

  pub_change_notification =
      nh->advertise<std_msgs::Empty>("/odin/internal/model_changed", 1);

  original_table_projector_position = {
      cv::Point2f(795, 119), cv::Point2f(856, 141), cv::Point2f(803, 247),
      cv::Point2f(728, 221)};

  // Create a timer
  interaction_timer_ = nh->createTimer(
      ros::Duration(0.5), &ProjectorInterfaceModel::reset_interactions, this);

  startTime = ros::Time::now();

  left_hand = std::make_shared<Hand>("left");
  right_hand = std::make_shared<Hand>("right");

  right_hand_detected = false;
  left_hand_detected = false;
  hands_detected = false;
  original_max_width = 0;
  original_max_height = 0;
}

void ProjectorInterfaceModel::create_border_layout(
    int rows, int cols, float sf_factor, bool adjacent,
    std_msgs::ColorRGBA status_booked, std_msgs::ColorRGBA status_free,
    std_msgs::ColorRGBA status_operator) {
  for (auto &zone : zones) {
    if (zone->name == "shelf" || zone->name == "table") {
      zone->create_border_layout(rows, cols, sf_factor, adjacent, status_booked,
                                 status_free, status_operator);
    }
    break;
  }
}

void ProjectorInterfaceModel::reset_interactions(const ros::TimerEvent &) {
  if (!hands_detected) {
    if (!action_triggered) {
      // Check for interaction
      for (auto &zone : zones) {
        zone->resetInteractions();
      }

      // Notify controller
      action_triggered = true;
    }
  } else {
    // Reset the variable for the next check
    hands_detected = false;
  }
  notify();
}

void ProjectorInterfaceModel::reset_interactions() {
  for (auto &zone : zones) {
    zone->resetInteractions();
  }
}

void ProjectorInterfaceModel::add_zone(
    std::shared_ptr<DisplayArea> display_area,
    std::vector<geometry_msgs::Point> camera_frame) {
  if (display_area->name == "cell_left" || display_area->name == "cell_right") {
    std::vector<cv::Point> projector_frame;
    for (size_t i = 0; i < camera_frame.size(); ++i) {
      projector_frame.push_back(
          cv::Point(camera_frame[i].x, camera_frame[i].y));
    }

    display_area->setProjectorFrame(projector_frame);
    display_area->filling = true;
  } else {
    display_area->setCameraFrame(camera_frame);

    std::vector<cv::Point> projector_frame;

    projector_frame.push_back(fromCamera2Projector(camera_frame[0]));
    projector_frame.push_back(fromCamera2Projector(camera_frame[1]));
    projector_frame.push_back(fromCamera2Projector(camera_frame[2]));
    projector_frame.push_back(fromCamera2Projector(camera_frame[3]));

    display_area->setProjectorFrame(projector_frame);

    std::vector<geometry_msgs::Point> robot_frame_points;

    robot_frame_points.push_back(fromPixel2Robot(camera_frame[0]).position);
    robot_frame_points.push_back(fromPixel2Robot(camera_frame[1]).position);
    robot_frame_points.push_back(fromPixel2Robot(camera_frame[2]).position);
    robot_frame_points.push_back(fromPixel2Robot(camera_frame[3]).position);

    display_area->setRobotFrame(robot_frame_points);
  }

  zones.push_back(display_area);
}

void ProjectorInterfaceModel::addInstructions(std::string zone,
                                              std::string title,
                                              std_msgs::ColorRGBA title_color) {
  ROS_INFO("%s NEED TO ADD %s", zone.c_str(), title.c_str());
  auto search = title.find("active");
  if (search != std::string::npos) {
    title = "Light curtain|reset needed";
    for (auto &z : zones) {
      if (z->name == zone) {
        z->instructions = title;
        break;
      }
    }
    notify();
  }

  search = title.find("out");
  if (search != std::string::npos) {
    title = "Watch out|light curtain";
    for (auto &z : zones) {
      if (z->name == zone) {
        z->instructions = title;
        break;
      }
    }
    notify();
  }

  search = title.find("needed");
  if (search != std::string::npos) {
    title = "Light curtain|reset needed";
  }
}

void ProjectorInterfaceModel::addButton(
    std::string request_id, std::string zone, std::string name,
    std::string text, std_msgs::ColorRGBA button_color,
    std_msgs::ColorRGBA text_color, geometry_msgs::Pose center, float radius) {
  for (auto &z : zones) {
    if (z->name == zone) {
      std::shared_ptr<Button> btn = std::make_shared<Button>(
          nh_, request_id, name, text, button_color, text_color, radius);
      geometry_msgs::Pose transformed_center =
          z->compute_absolute_world_position(center);
      btn->center_cam_point = cv::Point(transformed_center.position.x,
                                        transformed_center.position.y);
      btn->center = fromPixel2Robot(transformed_center.position);
      btn->center_projected_point =
          fromCamera2SmartInterface(transformed_center.position);
      btn->setXratio(center.position.x);
      btn->setYratio(center.position.y);
      z->addButton(btn);
      break;
    }
  }
  notify();
}

void ProjectorInterfaceModel::change_button_color(
    std::string resource_id, std_msgs::ColorRGBA button_color) {
  for (auto &z : zones) {
    if (z->change_button_color(resource_id, button_color)) {
      notify();
      break;
    }
  }
}

void ProjectorInterfaceModel::addStaticBorder(
    cv::Mat depth_img, std::string r_id, std::string z, int pos_row,
    int pos_col, geometry_msgs::PolygonStamped bord, std::string b_topic,
    std_msgs::ColorRGBA b_color, bool filling, int thic, ros::Duration life,
    bool track) {
  ROS_INFO("adding border");
  ROS_INFO("%s", z.c_str());
  for (auto &zone : zones) {
    ROS_INFO("%s", zone->name.c_str());

    if (zone->name == z && !zone->containsBorder(r_id)) {
      std::shared_ptr<StaticBorder> sb = std::make_shared<StaticBorder>(
          nh_, r_id, pos_row, pos_col, bord, b_topic, b_color, filling, thic,
          life, track);
      if (zone->border_layout.rows == 0 && zone->border_layout.cols == 0) {
        ROS_INFO("the old way");
        // Also compute camera coordinates for detections
        // TOP LEFT
        geometry_msgs::Pose border_top_left_pose;
        border_top_left_pose.position = sb->topLeftCornerPt;
        sb->top_left_cam_point = fromRobot2Pixel(border_top_left_pose);

        // BOTTOM RIGHT
        geometry_msgs::Pose border_bottom_right_pose;
        border_bottom_right_pose.position = sb->bottomRightCornerPt;
        sb->bottom_right_cam_point = fromRobot2Pixel(border_bottom_right_pose);

        sb->roi_rect =
            cv::Rect(sb->top_left_cam_point, sb->bottom_right_cam_point);
        sb->baseline = depth_img(sb->roi_rect);

        geometry_msgs::Point tl, br;
        tl.x = sb->top_left_cam_point.x;
        tl.y = sb->top_left_cam_point.y;

        br.x = sb->bottom_right_cam_point.x;
        br.y = sb->bottom_right_cam_point.y;

        sb->top_left_proj_point = fromCamera2Projector(tl);
        sb->bottom_right_proj_point = fromCamera2Projector(br);
      } else {
        ROS_INFO("the new way");
        std::vector<cv::Point> border_coordinates =
            zone->generate_border(pos_row, pos_col);

        sb->top_left_proj_point = border_coordinates[0];
        sb->top_right_proj_point = border_coordinates[1];
        sb->bottom_right_proj_point = border_coordinates[2];
        sb->bottom_left_proj_point = border_coordinates[3];

        sb->top_left_cam_point = fromProjector2Camera(sb->top_left_proj_point);
        sb->top_right_cam_point =
            fromProjector2Camera(sb->top_right_proj_point);
        sb->bottom_right_cam_point =
            fromProjector2Camera(sb->bottom_right_proj_point);
        sb->bottom_left_cam_point =
            fromProjector2Camera(sb->bottom_left_proj_point);

        sb->roi_rect =
            cv::Rect(sb->top_left_cam_point, sb->bottom_right_cam_point);
        sb->baseline = depth_img(sb->roi_rect);

        /*
        geometry_msgs::Point tl, tr, br, bl;
        tl.x = sb->top_left_cam_point.x;
        tl.y = sb->top_left_cam_point.y;
        sb->topLeftCornerPt = fromPixel2Robot(tl);

        tl.x = sb->top_right_cam_point.x;
        tl.y = sb->top_right_cam_point.y;
        sb->topRightCornerPt = fromPixel2Robot(tr);

        br.x = sb->bottom_right_cam_point.x;
        br.y = sb->bottom_right_cam_point.y;
        sb->bottomRightCornerPt = fromPixel2Robot(br);

        bl.x = sb->bottom_left_cam_point.x;
        bl.y = sb->bottom_left_cam_point.y;
        sb->bottomLeftCornerPt = fromPixel2Robot(bl);
        */
      }

      zone->addBorder(sb);

      break;
    }
  }
  // Notify controller
  notify();
}

void ProjectorInterfaceModel::addDynamicBorder(std::string r_id, std::string z,
                                               std::string b_topic,
                                               std_msgs::ColorRGBA b_color,
                                               bool filling, int thic,
                                               ros::Duration life, bool track) {

}

void ProjectorInterfaceModel::robot_book_border(std::string id) {
  for (auto &z : zones) {
    if (z->robot_book_border(id)) {
      notify();
      break;
    }
  }
}

void ProjectorInterfaceModel::robot_release_border(std::string id, int status) {
  for (auto &z : zones) {
    if (z->robot_release_border(id, status)) {
      notify();
      break;
    }
  }
}

void ProjectorInterfaceModel::operator_book_border(std::string id) {
  for (auto &z : zones) {
    if (z->operator_book_border(id)) {
      notify();
      break;
    }
  }
}

void ProjectorInterfaceModel::operator_release_border(std::string id,
                                                      int status) {
  for (auto &z : zones) {
    if (z->operator_release_border(id, status)) {
      notify();
      break;
    }
  }
}

void ProjectorInterfaceModel::notify() {
  pub_change_notification.publish(std_msgs::Empty());
}

cv::Point2f ProjectorInterfaceModel::getProjectionCenter(
    std::vector<cv::Point2f> projection_area) {
  double centerX = (projection_area[0].x + projection_area[1].x +
                    projection_area[2].x + projection_area[3].x) /
                   4.0;
  double centerY = (projection_area[0].y + projection_area[1].y +
                    projection_area[2].y + projection_area[3].y) /
                   4.0;

  cv::Point2f center(centerX, centerY);
  return center;
}

cv::Size2f ProjectorInterfaceModel::getProjectionSize(
    std::vector<cv::Point2f> projection_area) {
  cv::Size2f size;

  // Calculate side lengths
  double width1 = std::hypot(projection_area[1].x - projection_area[0].x,
                             projection_area[1].y - projection_area[0].y);
  double width2 = std::hypot(projection_area[2].x - projection_area[3].x,
                             projection_area[2].y - projection_area[3].y);
  double height1 = std::hypot(projection_area[3].x - projection_area[0].x,
                              projection_area[3].y - projection_area[0].y);
  double height2 = std::hypot(projection_area[2].x - projection_area[1].x,
                              projection_area[2].y - projection_area[1].y);

  // Find maximum width and height

  double max_width = std::max(width1, width2);
  double max_height = std::max(height1, height2);

  if (original_max_width == 0 && original_max_height == 0) {
    original_max_width = max_width;
    original_max_height = max_height;
  }

  if (isPortrait(max_width, max_height)) {
    // Swap if needed so that max_height is the vertical and max_width is
    // horizontal
    std::swap(original_max_width, original_max_height);
  }
  // always return the same sizes but swap them if orientation changes
  size = cv::Size2f(original_max_width, original_max_height);
  return size;
}

double ProjectorInterfaceModel::getProjectionAngle(
    cv::Point2f center, cv::Size2f size,
    std::vector<cv::Point2f> projection_area,
    std::shared_ptr<DisplayArea> zone) {
  int top_left_straight_table_x, top_left_straight_table_y,
      top_right_straight_table_x, top_right_straight_table_y;

  top_left_straight_table_x = center.x - size.width / 2;
  top_left_straight_table_y = center.y - size.height / 2;
  top_right_straight_table_x = center.x + size.width / 2;
  top_right_straight_table_y = center.y - size.height / 2;

  Eigen::Vector2d point1(top_left_straight_table_x, top_left_straight_table_y);
  Eigen::Vector2d point2(top_right_straight_table_x,
                         top_right_straight_table_y);
  Eigen::Vector2d point3(projection_area[0].x, projection_area[0].y);
  Eigen::Vector2d point4(projection_area[1].x, projection_area[1].y);

  Eigen::Vector2d direction1 = point2 - point1;
  Eigen::Vector2d direction2 = point4 - point3;

  Eigen::Vector2d normLine1 = direction1.normalized();
  Eigen::Vector2d normLine2 = direction2.normalized();

  double cosTheta = normLine1.dot(normLine2);
  cosTheta = boost::algorithm::clamp(cosTheta, -1.0, 1.0);

  double angleRadians = std::acos(cosTheta);
  zone->projector_rotation_angle = angleRadians;

  double angleDegrees = angleRadians * (180.0 / M_PI);

  double crossProductZ =
      normLine1.x() * normLine2.y() - normLine1.y() * normLine2.x();
  if (crossProductZ < 0) {
    angleDegrees = -angleDegrees;
  }
  return angleDegrees;
}

bool ProjectorInterfaceModel::isPortrait(double width, double height) {
  return height > width;
}

void ProjectorInterfaceModel::updateMovingTable(
    const tuni_whitegoods_msgs::DynamicArea &moving_table) {
  for (auto &zone : zones) {
    if (zone->name == "moving_table") {
      auto convertToPoint =
          [](const boost::array<double, 2> &arr) -> geometry_msgs::Point {
        geometry_msgs::Point point;
        point.x = arr[0];
        point.y = arr[1];
        point.z = 2.2;
        return point;
      };

      std::vector<geometry_msgs::Point> camera_frame;
      // Convert each corner and add to camera_frame
      camera_frame.push_back(convertToPoint(moving_table.top_left));
      camera_frame.push_back(convertToPoint(moving_table.top_right));
      camera_frame.push_back(convertToPoint(moving_table.bottom_right));
      camera_frame.push_back(convertToPoint(moving_table.bottom_left));

      zone->setCameraFrame(camera_frame);

      std::vector<cv::Point2f> projector_frame;
      for (size_t i = 0; i < camera_frame.size(); ++i) {
        projector_frame.push_back(fromCamera2SmartInterface(camera_frame[i]));
      }

      cv::Point2f center = getProjectionCenter(projector_frame);
      cv::Size2f size = getProjectionSize(projector_frame);
      double angleDegrees =
          getProjectionAngle(center, size, projector_frame, zone);

      cv::RotatedRect movingTable(center, size, angleDegrees);

      cv::Point2f vertices[4];
      movingTable.points(vertices);

      std::vector<cv::Point> proj_frame;

      proj_frame.push_back(cv::Point(static_cast<int>(vertices[0].x),
                                     static_cast<int>(vertices[0].y)));
      proj_frame.push_back(cv::Point(static_cast<int>(vertices[1].x),
                                     static_cast<int>(vertices[1].y)));
      proj_frame.push_back(cv::Point(static_cast<int>(vertices[2].x),
                                     static_cast<int>(vertices[2].y)));
      proj_frame.push_back(cv::Point(static_cast<int>(vertices[3].x),
                                     static_cast<int>(vertices[3].y)));
      zone->setProjectorFrame(proj_frame);

      std::vector<geometry_msgs::Point> robot_frame_points;

      robot_frame_points.push_back(fromPixel2Robot(camera_frame[0]).position);
      robot_frame_points.push_back(fromPixel2Robot(camera_frame[1]).position);
      robot_frame_points.push_back(fromPixel2Robot(camera_frame[2]).position);
      robot_frame_points.push_back(fromPixel2Robot(camera_frame[3]).position);
      zone->setRobotFrame(robot_frame_points);

      std::vector<std::shared_ptr<Button>> buttons;
      zone->fetchButtons(buttons);

      for (auto &button : buttons) {
        geometry_msgs::Pose center;
        center.position.x = button->x_ratio;
        center.position.y = button->y_ratio;
        geometry_msgs::Pose transformed_center =
            zone->compute_absolute_world_position(center);
        button->center_cam_point.x = transformed_center.position.x;
        button->center_cam_point.y = transformed_center.position.y;
        button->center = fromPixel2Robot(transformed_center.position);
        button->center_projected_point =
            fromCamera2SmartInterface(transformed_center.position);
      }
    }
  }
}

bool ProjectorInterfaceModel::updateHandPose(
    const std::string &name, const geometry_msgs::Point &position) {
  hands_detected = true;
  if (hand_visualization) {
    geometry_msgs::Pose hand_pose = fromPixel2Robot(position);

    if (name == "left") {
      left_hand->set_position(position, hand_pose.position);
    } else if (name == "right") {
      right_hand->set_position(position, hand_pose.position);
    }
  }

  // Check for interaction
  bool interaction = false;
  for (auto &zone : zones) {
    if (zone->checkForInteractions(name, position)) {
      interaction = true;
      notify();
      break;
    }
  }

  return interaction;
}

std::vector<std::shared_ptr<Button>> ProjectorInterfaceModel::getButtons() {
  std::vector<std::shared_ptr<Button>> buttons;
  for (auto &zone : zones) {
    zone->fetchButtons(buttons);
  }
  return buttons;
}

std::vector<std::shared_ptr<StaticBorder>>
ProjectorInterfaceModel::getBorders() {
  std::vector<std::shared_ptr<StaticBorder>> borders;
  for (auto &zone : zones) {
    zone->fetchBorders(borders);
  }
  return borders;
}

std::vector<std::shared_ptr<Hand>> ProjectorInterfaceModel::getHands() {
  std::vector<std::shared_ptr<Hand>> hands;
  hands.push_back(left_hand);
  hands.push_back(right_hand);
  return hands;
}

std::vector<std::shared_ptr<DisplayArea>>
ProjectorInterfaceModel::getDisplayAreas() {
  return zones;
}

cv::Point ProjectorInterfaceModel::fromRobot2Pixel(geometry_msgs::Pose pose) {
  geometry_msgs::PoseStamped in_point_stamped;
  tuni_whitegoods_msgs::TransformRobotCameraCoordinates srv_pose;
  tuni_whitegoods_msgs::Transform3DToPixel srv_3D_to_pixel;
  // Convert to 3D camera coordinates frame
  in_point_stamped.header.frame_id = "base";
  in_point_stamped.header.stamp = ros::Time(0);
  in_point_stamped.pose = pose;

  srv_pose.request.in_point_stamped = in_point_stamped;
  srv_pose.request.target_frame = "rgb_camera_link";

  if (!client_world_coordinates.call(srv_pose)) {
    ROS_ERROR("Failed to call service");
  }

  // Project to 2D image coordinates
  srv_3D_to_pixel.request.x =
      srv_pose.response.out_point_stamped.pose.position.x;
  srv_3D_to_pixel.request.y =
      srv_pose.response.out_point_stamped.pose.position.y;
  srv_3D_to_pixel.request.z =
      srv_pose.response.out_point_stamped.pose.position.z;
  if (!client_3D_to_pixel.call(srv_3D_to_pixel)) {
    ROS_ERROR("Failed to call service");
  }

  cv::Point result(srv_3D_to_pixel.response.u, srv_3D_to_pixel.response.v);

  return result;
}

geometry_msgs::Pose ProjectorInterfaceModel::fromPixel2Robot(
    geometry_msgs::Point pixel) {
  // Project pixel position to 3D
  tuni_whitegoods_msgs::TransformPixelTo3D srv_pixel_to_3D;
  srv_pixel_to_3D.request.u = pixel.x;
  srv_pixel_to_3D.request.v = pixel.y;
  srv_pixel_to_3D.request.depth = pixel.z;
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

  return srv.response.out_point_stamped.pose;
}

cv::Point ProjectorInterfaceModel::fromProjector2Camera(cv::Point pixel) {
  tuni_whitegoods_msgs::TransformPixelToProjection srv_projector_to_camera;
  srv_projector_to_camera.request.u = pixel.x;
  srv_projector_to_camera.request.v = pixel.y;
  client_reverse_projector_point.call(srv_projector_to_camera);

  cv::Point result;
  result.x = srv_projector_to_camera.response.u_prime;
  result.y = srv_projector_to_camera.response.v_prime;

  return result;
}

cv::Point ProjectorInterfaceModel::fromCamera2Projector(
    geometry_msgs::Point pixel) {
  tuni_whitegoods_msgs::TransformPixelToProjection srv_camera_to_projector;
  srv_camera_to_projector.request.u = pixel.x;
  srv_camera_to_projector.request.v = pixel.y;
  client_projector_point.call(srv_camera_to_projector);

  cv::Point result;
  result.x = srv_camera_to_projector.response.u_prime;
  result.y = srv_camera_to_projector.response.v_prime;

  return result;
}

cv::Point ProjectorInterfaceModel::fromCamera2SmartInterface(
    geometry_msgs::Point pixel) {
  tuni_whitegoods_msgs::TransformPixelToProjection srv_camera_to_projector;
  srv_camera_to_projector.request.u = pixel.x;
  srv_camera_to_projector.request.v = pixel.y;
  client_projector_smart_interface.call(srv_camera_to_projector);

  cv::Point result;
  result.x = srv_camera_to_projector.response.u_prime;
  result.y = srv_camera_to_projector.response.v_prime;

  return result;
}
