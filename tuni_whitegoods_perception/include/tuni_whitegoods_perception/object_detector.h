#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class StaticBorder;  // Forward declaration for StaticBorder class

class ObjectDetector {
 public:
  ObjectDetector();  // Constructor
  bool scan(cv::Mat image, cv::Rect roi);

 private:
  ros::NodeHandle* nh_;
};

#endif  // OBJECT_DETECTOR_H