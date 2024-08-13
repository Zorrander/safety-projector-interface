#include "tuni_whitegoods_view/camera_view.h"

#include <sensor_msgs/image_encodings.h>


using namespace std;


CameraView::CameraView()
{
  img_callback = nh_.subscribe("/depth_to_rgb/image_raw", 1,  &CameraView::depthSceneCallback,this);
}


void CameraView::depthSceneCallback(const sensor_msgs::ImageConstPtr& depth_msg){
  cv_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::BGR8);
  // Normalize the depth map to the range 0-255 for better visualization
  cv::normalize(cv_depth->image, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
  // Apply a color map
  cv::applyColorMap(depth_normalized, depth_colormap, cv::COLORMAP_JET);
}


