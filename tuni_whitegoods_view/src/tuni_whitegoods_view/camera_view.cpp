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
  publish_image();
}

void CameraView::updateButtons(
    const std::vector<std::shared_ptr<Button>>& buttons) {
  for (auto& button : buttons) {
    cv::circle(depth_normalized, button->center_cam_point, button->radius,
               button->btn_color, -1);
  }
  publish_image();
}

void CameraView::updateBorders(
    const std::vector<std::shared_ptr<StaticBorder>>& borders) {
  for (auto& border : borders) {
    cv::rectangle(
        depth_normalized, border->top_left_cam_point,
        border->bottom_right_cam_point,
        cv::Scalar(border->border_color.b * 255, border->border_color.g * 255,
                   border->border_color.r * 255),
        border->thickness * 2, cv::LINE_8);
  }

  publish_image();
}

void CameraView::updateHands(const std::vector<std::shared_ptr<Hand>>& hands) {
  for (auto& hand : hands) {
    cv::Point hand_position(hand->pixel_position.x, hand->pixel_position.y);
    cv::circle(depth_normalized, hand_position, 25, cv::Scalar(255, 255, 0),
               -1);
  }
  publish_image();
}

void CameraView::publish_image() {
  camera_viz_msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_normalized)
          .toImageMsg();
  viz_pub.publish(camera_viz_msg);
}

void CameraView::depthSceneCallback(
    const sensor_msgs::ImageConstPtr& depth_msg) {
  cv_depth =
      cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  // Normalize the depth map to the range 0-255 for better visualization
  cv::normalize(cv_depth->image, depth_normalized, 0, 255, cv::NORM_MINMAX,
                CV_8U);
  // Apply a color map
  // cv::applyColorMap(depth_normalized, depth_colormap, cv::COLORMAP_JET);
}
