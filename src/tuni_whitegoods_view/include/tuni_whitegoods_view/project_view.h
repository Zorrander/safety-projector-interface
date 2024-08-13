#ifndef Projector_H
#define Projector_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tuni_whitegoods_msgs/Projection.h>

class Projector
{
private:
  ros::NodeHandle nh_;
  cv_bridge::CvImagePtr cv_ptr;
  ros::Subscriber transform_sub;
  int shift;
  cv::Matx33d hom;
  cv::Mat sum_img;

public:
    Projector();
    ~Projector();
    void transformProject(const tuni_whitegoods_msgs::Projection::ConstPtr& msg);
};

#endif