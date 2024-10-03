#include "tuni_whitegoods_perception/object_detector.h"

ObjectDetector::ObjectDetector() {}

// Callback function for RGB image

bool ObjectDetector::scan(cv::Mat image, cv::Rect roi) {
  bool result = false;
  cv::Scalar color(0, 255, 0);  // Color for drawing (green)

  // Convert the BGR image to grayscale
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

  // Optional: Apply a threshold to binarize the image (depends on your use
  // case)
  cv::Mat binary_image;
  cv::threshold(gray_image, binary_image, 150, 255, cv::THRESH_BINARY);

  // Get the region of interest from the binary image
  cv::Mat roi_image = binary_image(roi);
  cv::Mat test_image = image(roi);

  // Find contours in the ROI
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(roi_image, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  // Check if more than one contour is found
  if (contours.size() > 1) {
    ROS_INFO("Object detected");
    result = true;
  }

  // cv::drawContours(test_image, contours, -1, (0,0,255), 3);
  // cv::imshow("Input", test_image);
  // cv::waitKey(0);

  return result;
}

/*
// Callback function for depth image
bool ObjectDetector::scan(cv::Mat image, cv::Rect roi) {
    bool result = false;
    cv::Mat roi_depthmap = image(roi);
    // Apply the depth range threshold
    cv::Mat mask;
    cv::Scalar min_depth = cv::Scalar(1400);
    cv::Scalar max_depth = cv::Scalar(1500);
    cv::inRange(roi_depthmap, min_depth, max_depth, mask);

    // Find contours in the mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
cv::CHAIN_APPROX_SIMPLE);

    cv::Mat depth_normalized, depth_colormap;

    cv::normalize(roi_depthmap, depth_normalized, 0, 255, cv::NORM_MINMAX,
                    CV_8U);

    cv::applyColorMap(depth_normalized, depth_colormap, cv::COLORMAP_JET);

    cv::drawContours(depth_colormap, contours, -1, (0,255,0), 3);
    cv::imshow("Input", depth_colormap);
    cv::waitKey(0);

    if (contours.size() > 1) {
        ROS_INFO("Object detected");
        result = true;
    }
    return result;
}


*/