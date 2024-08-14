#ifndef CameraView_H
#define CameraView_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include "tuni_whitegoods_view/view.h"

class CameraView : public View
{
private:
  ros::NodeHandle nh_;
  cv_bridge::CvImagePtr cv_depth;
  ros::Subscriber img_callback;
  cv::Mat depth_normalized, depth_colormap;

  void depthSceneCallback(const sensor_msgs::ImageConstPtr& depth_msg);

public:
    CameraView();
    void init() override;
    void update() override;
    
};

#endif