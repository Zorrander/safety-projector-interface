#include "tuni_whitegoods_view/camera_view.h"

#include <sensor_msgs/image_encodings.h>

using namespace std;

CameraView::CameraView(ros::NodeHandle* nh) : it_(*nh), nh_(nh) {
  img_callback = it_.subscribe("/depth_to_rgb/image_raw", 10,
                               &CameraView::depthSceneCallback, this);
  viz_pub = it_.advertise("/odin/visualization/camera_view", 10);

  ROS_INFO("CameraView running");
}

void CameraView::init(std::vector<std::shared_ptr<DisplayArea>> zones) {
  for (auto& zone : zones) {
    cv::Point tl(zone->camera_frame_area[0].x, zone->camera_frame_area[0].y);
    cv::Point tr(zone->camera_frame_area[1].x, zone->camera_frame_area[1].y);
    cv::Point br(zone->camera_frame_area[2].x, zone->camera_frame_area[2].y);
    cv::Point bl(zone->camera_frame_area[3].x, zone->camera_frame_area[3].y);

    std::vector<cv::Point> rectanglePoints = {tl, tr, br, bl};

    const cv::Scalar color(255, 255, 255);
    const int thickness = 10;

    cv::polylines(cv_depth, rectanglePoints, true, color, thickness,
                  cv::LINE_8);
  }
  publish_image();
}

void CameraView::updateButtons(
    const std::vector<std::shared_ptr<Button>>& buttons) {
  for (auto& button : buttons) {
    cv::circle(cv_depth, button->center_cam_point, button->radius,
               button->btn_color, -1);
  }
  publish_image();
}

void CameraView::updateBorders(
    const std::vector<std::shared_ptr<StaticBorder>>& borders) {
  for (auto& border : borders) {
    cv::rectangle(
        cv_depth, border->top_left_cam_point, border->bottom_right_cam_point,
        cv::Scalar(border->border_color.b * 255, border->border_color.g * 255,
                   border->border_color.r * 255),
        border->thickness * 2, cv::LINE_8);
  }

  publish_image();
}

void CameraView::updateHands(const std::vector<std::shared_ptr<Hand>>& hands) {
  for (auto& hand : hands) {
    cv::Point hand_position(hand->pixel_position.x, hand->pixel_position.y);
    cv::circle(cv_depth, hand_position, 25, cv::Scalar(255, 255, 0), -1);
  }
  publish_image();
}

void CameraView::updateDisplayAreas(
    const std::vector<std::shared_ptr<DisplayArea>>& zones) {
  for (auto& zone : zones) {
    ROS_INFO("Updating (%s)", zone->name.c_str());
    cv::Point tl(zone->camera_frame_area[0].x, zone->camera_frame_area[0].y);
    cv::Point tr(zone->camera_frame_area[1].x, zone->camera_frame_area[1].y);
    cv::Point br(zone->camera_frame_area[2].x, zone->camera_frame_area[2].y);
    cv::Point bl(zone->camera_frame_area[3].x, zone->camera_frame_area[3].y);

    std::vector<cv::Point> rectanglePoints = {tl, tr, br, bl};

    const cv::Scalar color(255, 255, 255);
    const int thickness = 10;

    cv::polylines(cv_depth, rectanglePoints, true, color, thickness,
                  cv::LINE_8);
  }
  publish_image();
}

void CameraView::publish_image() {
  // Normalize the depth map to the range 0-255 for better visualization
  cv::normalize(cv_depth, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
  // Apply a color map
  cv::applyColorMap(depth_normalized, depth_colormap, cv::COLORMAP_JET);
  camera_viz_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_colormap)
          .toImageMsg();
  viz_pub.publish(camera_viz_msg);
}

void CameraView::depthSceneCallback(
    const sensor_msgs::ImageConstPtr& depth_msg) {
  bridge_cv_depth =
      cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  cv_depth = bridge_cv_depth->image;
}
