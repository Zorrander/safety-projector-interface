#include "tuni_whitegoods_perception/object_detector.h"

ObjectDetector::ObjectDetector() {}

bool ObjectDetector::scan(cv::Mat depth_image, cv::Mat baseline) {
  bool result = false;

  cv::Mat difference;
  cv::absdiff(depth_image, baseline, difference);

  int threshold_value = 10;
  cv::Mat thresh;
  cv::threshold(difference, thresh, threshold_value, 255, cv::THRESH_BINARY);

  int non_zero_count = cv::countNonZero(thresh);

  if (non_zero_count > 0) {
    ROS_INFO("Object detected! Changed pixels: ");
    result = true;
  } else {
    ROS_INFO("No significant object detected.");
  }

  /*
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
  */

  return result;
}
