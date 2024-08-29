#include "tuni_whitegoods_view/camera_view.h"

#include <sensor_msgs/image_encodings.h>

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

CameraView::CameraView(ros::NodeHandle *nh) : it_(*nh), nh_(nh) {
  img_callback = it_.subscribe("/depth_to_rgb/image_raw", 10,
                               &CameraView::depthSceneCallback, this);
  viz_pub = it_.advertise("/odin/visualization/camera_view", 1);
  cv::namedWindow(OPENCV_WINDOW);
  ROS_INFO("CameraView running");
}

CameraView::~CameraView() { cv::destroyWindow(OPENCV_WINDOW); }

void CameraView::init() {
  camera_viz_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_colormap)
          .toImageMsg();
  viz_pub.publish(camera_viz_msg);
}

void CameraView::updateButtons(std::vector<std::shared_ptr<Button>> buttons) {
  camera_viz_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_colormap)
          .toImageMsg();
  viz_pub.publish(camera_viz_msg);
}

void CameraView::updateBorders(std::vector<std::shared_ptr<StaticBorder>> borders) {
  camera_viz_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_colormap)
          .toImageMsg();
  viz_pub.publish(camera_viz_msg);
}

void CameraView::updateHands(std::vector<std::shared_ptr<Hand>> hands) {
  camera_viz_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_colormap)
          .toImageMsg();
  viz_pub.publish(camera_viz_msg);
}

void CameraView::depthSceneCallback(
    const sensor_msgs::ImageConstPtr &depth_msg) {
  cv_depth =
      cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  // Normalize the depth map to the range 0-255 for better visualization
  cv::normalize(cv_depth->image, depth_normalized, 0, 255, cv::NORM_MINMAX,
                CV_8U);
  // Apply a color map
  cv::applyColorMap(depth_normalized, depth_colormap, cv::COLORMAP_JET);
}
