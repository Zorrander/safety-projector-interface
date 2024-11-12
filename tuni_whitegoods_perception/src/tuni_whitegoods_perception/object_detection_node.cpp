#include "tuni_whitegoods_perception/object_detector.h"

ObjectDetector::ObjectDetector(ros::NodeHandle* nh) : nh_(nh) {
  non_zero_count_threshold = 10;
  threshold_value = 10;
  kernel_size = 3;
  threshold_sub = nh_->subscribe("/odin/object_detection/set_threshold", 1,
                                 &ObjectDetector::thresholdCallback, this);
  non_zero_threshold_sub =
      nh_->subscribe("/odin/object_detection/set_non_zero_threshold", 1,
                     &ObjectDetector::nonZeroThresholdCallback, this);
  noise_reduction_sub =
      nh_->subscribe("/odin/object_detection/set_noise_reduction", 1,
                     &ObjectDetector::noiseReductionCallback, this);
}

bool ObjectDetector::scan(cv::Mat depth_image, cv::Mat baseline) {
  bool result = false;
  ROS_INFO("comparing...");
  cv::Mat difference;
  cv::absdiff(depth_image, baseline, difference);

  cv::Mat thresh;
  cv::threshold(difference, thresh, threshold_value, 255, cv::THRESH_BINARY);

  int kernel_size = 3;
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
  cv::morphologyEx(thresh, thresh, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(thresh, thresh, cv::MORPH_CLOSE, kernel);

  int non_zero_count = cv::countNonZero(thresh);

  if (non_zero_count > non_zero_count_threshold) {
    ROS_INFO("Object detected! Changed pixels: ");
    result = true;
  } else {
    ROS_INFO("No significant object detected.");
  }

  
  cv::Mat depth_normalized, depth_colormap;
  cv::Mat baseline_normalized, baseline_colormap;
  cv::Mat difference_normalized, difference_colormap;

  cv::normalize(depth_image, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
  cv::applyColorMap(depth_normalized, depth_colormap, cv::COLORMAP_JET);

  cv::normalize(baseline, baseline_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
  cv::applyColorMap(baseline_normalized, baseline_colormap, cv::COLORMAP_JET);

  cv::normalize(thresh, difference_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
  cv::applyColorMap(difference_normalized, difference_colormap,
                    cv::COLORMAP_JET);

  std::vector<cv::Mat> images = {depth_colormap, baseline_colormap,
                                 difference_colormap};
  cv::Mat combined;
  cv::hconcat(images, combined);
  cv::namedWindow("Object detection", cv::WINDOW_AUTOSIZE);
  cv::imshow("Object detection", combined);

  cv::waitKey(0);
  

  return result;
}

void ObjectDetector::thresholdCallback(const std_msgs::Int32::ConstPtr& msg) {
  threshold_value = msg->data;
}

void ObjectDetector::nonZeroThresholdCallback(
    const std_msgs::Int32::ConstPtr& msg) {
  non_zero_count_threshold = msg->data;
}

void ObjectDetector::noiseReductionCallback(
    const std_msgs::Int32::ConstPtr& msg) {
  kernel_size = msg->data;
}
