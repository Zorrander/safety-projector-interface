#ifndef CameraView_H
#define CameraView_H

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "tuni_whitegoods_view/view.h"


class CameraView : public View
{
private:
  ros::NodeHandle* nh_;
  image_transport::ImageTransport it_;
  cv_bridge::CvImagePtr cv_depth;

  image_transport::Subscriber img_callback;
  image_transport::Publisher viz_pub;
  sensor_msgs::ImagePtr camera_viz_msg;

  cv::Mat depth_normalized, depth_colormap;

  void depthSceneCallback(const sensor_msgs::ImageConstPtr& depth_msg);

public:
    CameraView(ros::NodeHandle* nh);
    ~CameraView();
    void init() override;
    void update() override;
    
};

#endif