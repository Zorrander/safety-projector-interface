#include "tuni_whitegoods_perception/object_detector.h"

ObjectDetector::ObjectDetector() {}

// Callback function for RGB image

// Callback function for depth image
bool ObjectDetector::scan(cv::Mat depth_image, cv::Mat baseline) {
  bool result = false;

  cv::Mat difference;
  cv::absdiff(depth_image, baseline, difference);

  int threshold_value = 10;  // Adjust based on the sensitivity needed
  cv::Mat thresh;
  cv::threshold(difference, thresh, threshold_value, 255, cv::THRESH_BINARY);

  int non_zero_count = cv::countNonZero(thresh);

  if (non_zero_count > 0) {
    ROS_INFO("Object detected! Changed pixels: ");
    result = true;
  } else {
    ROS_INFO("No significant object detected.");
  }

  // Step 5 (optional): Display the result for visualization
  cv::imshow("Difference", thresh);
  cv::waitKey(0);  // Wait until a key is pressed

  return result;
}
