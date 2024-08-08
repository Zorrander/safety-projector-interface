/*
Class that project everything. An instance of this class is launched for each projector connected.
It basically receives a list of projection that contains everything to display (smart interface, borders...) 
each projection is an image with a transform to display them. 
For a system with only one camera and projector, it only need to run once.
*/

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>


using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class CameraView
{
  ros::NodeHandle nh_;
  cv_bridge::CvImagePtr cv_ptr;
  ros::Subscriber transform_sub;
  int shift;
  cv::Matx33d hom;

public:
  CameraView()
  {
    img_callback = nh_->subscribe("/depth_to_rgb/image_raw", 1, depthSceneCallback,this);
  }

  ~CameraView()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void depthSceneCallback(const sensor_msgs::ImageConstPtr& depth_msg){
    cv_bridge::CvImagePtr cv_viz_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::BGR8);
    cv_viz_depth = cv_viz_depth_ptr->image.clone();

         // Normalize the depth map to the range 0-255 for better visualization
     cv::Mat depth_normalized;
     cv::normalize(cv_depth, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
     // Apply a color map
     cv::Mat depth_colormap;
     cv::applyColorMap(depth_normalized, depth_colormap, cv::COLORMAP_JET);
  }

  void transformProject(const unity_msgs::ListDataProj::ConstPtr& msg)
  {

    // Draw contours 
    cv::Scalar color(255, 0, 255); 
    cv::drawContours(depth_colormap, contours, -1, color, 2);
    cv::circle(depth_colormap, cv::Point(sb->getCenter().x, sb->getCenter().y), 10, cv::Scalar(255, 0, 0), 2);
    // Draw ROI
    cv::rectangle(depth_colormap, roi_rect, cv::Scalar(255,255,255), 3, cv::LINE_8);
    
  }

};
