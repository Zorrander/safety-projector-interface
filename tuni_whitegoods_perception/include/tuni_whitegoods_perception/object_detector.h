#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>

#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ObjectDetector {
 public:
  ObjectDetector(ros::NodeHandle* nh);
  bool scan(cv::Mat depth_image, cv::Mat baseline);

 private:
  ros::NodeHandle* nh_;
  ros::Subscriber threshold_sub;
  ros::Subscriber non_zero_threshold_sub;
  ros::Subscriber noise_reduction_sub;
  void thresholdCallback(const std_msgs::Int32::ConstPtr& msg);
  void nonZeroThresholdCallback(const std_msgs::Int32::ConstPtr& msg);
  void noiseReductionCallback(const std_msgs::Int32::ConstPtr& msg);
  bool fileExists(const std::string& filename);

  void saveImageWithUniqueName(const cv::Mat& image, const std::string& dirPath,
                               const std::string& baseName);
  int threshold_value;
  int non_zero_count_threshold;
  int kernel_size;
};

#endif  // OBJECT_DETECTOR_H