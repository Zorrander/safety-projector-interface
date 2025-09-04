#include "tuni_whitegoods_perception/template_matcher.h"

#include <iostream>

TemplateMatcher::TemplateMatcher(const std::string& templatePath)
    : templatePath(templatePath) {
  templateImage0 = cv::imread(templatePath + "_0.png", cv::IMREAD_COLOR);
  templateImage1 = cv::imread(templatePath + "_1.png", cv::IMREAD_COLOR);
  templateImage2 = cv::imread(templatePath + "_2.png", cv::IMREAD_COLOR);
  templateImage3 = cv::imread(templatePath + "_3.png", cv::IMREAD_COLOR);
  templateImage4 = cv::imread(templatePath + "_4.png", cv::IMREAD_COLOR);
  templateImage5 = cv::imread(templatePath + "_5.png", cv::IMREAD_COLOR);

  templates.push_back(templateImage0);
  templates.push_back(templateImage1);
  templates.push_back(templateImage2);
  templates.push_back(templateImage3);
  templates.push_back(templateImage4);
  templates.push_back(templateImage5);

  methods = {cv::TM_CCOEFF,       cv::TM_CCOEFF_NORMED, cv::TM_CCORR,
             cv::TM_CCORR_NORMED, cv::TM_SQDIFF,        cv::TM_SQDIFF_NORMED};
}

cv::Mat TemplateMatcher::centerCropToMatchTemplate(const cv::Mat& input,
                                                   const cv::Mat& templ) {
  int centerX = input.cols / 2;
  int centerY = input.rows / 2;
  int w = templ.cols;
  int h = templ.rows;

  int x = std::max(0, centerX - w / 2);
  int y = std::max(0, centerY - h / 2);

  // Ensure ROI is within bounds
  if (x + w > input.cols || y + h > input.rows) {
    throw std::runtime_error(
        "Template size exceeds input image bounds after cropping.");
  }

  return input(cv::Rect(x, y, w, h));
}

bool TemplateMatcher::scan(cv::Mat image, cv::Rect roi) {
  double totalConfidence = 0.0;
  int detectedCount = 0;  // Number of templates that matched

  cv::Mat img = image.clone();
  cv::Mat result;

  for (size_t i = 0; i < templates.size(); ++i) {
    // Load the current template
    cv::Mat templateImage = templates[i];
    int w = templateImage.cols;
    int h = templateImage.rows;

    if (templateImage.cols > image.cols || templateImage.rows > image.rows) {
      double scale_x = static_cast<double>(image.cols) / templateImage.cols;
      double scale_y = static_cast<double>(image.rows) / templateImage.rows;
      double scale = std::min(scale_x, scale_y);

      cv::resize(templateImage, templateImage, cv::Size(), scale, scale);
    }

    cv::matchTemplate(image, templateImage, result, cv::TM_CCOEFF_NORMED);

    double minVal, maxVal;
    cv::Point minLoc, maxLoc, matchLoc;

    const double detectionThreshold = 0.6;  // Adjust threshold as needed

    for (int y = 0; y < result.rows; y++) {
      for (int x = 0; x < result.cols; x++) {
        float matchVal = result.at<float>(y, x);
        if (matchVal >= detectionThreshold) {
          cv::Point pt(x, y);
          cv::Point center(x + w / 2,
                           y + h / 2);  // Center of the matched region

          bool inside_roi = false;
          if (roi.contains(center)) {
            cv::rectangle(image, pt, cv::Point(pt.x + w, pt.y + h),
                          cv::Scalar(0, 255, 0),
                          2);  // Green for match inside ROI
            std::cout << "Match center at (" << center.x << ", " << center.y
                      << ") is inside ROI with confidence: " << matchVal
                      << std::endl;

            return true;
          }
        }
      }
    }
  }

  //cv::imshow("Display Window", image);
  //cv::waitKey(1);

  return false;
}

void TemplateMatcher::drawResult(const cv::Mat& img, const cv::Mat& result,
                                 const cv::Point& topLeft,
                                 const std::string& methodName) {
  cv::imshow("Matching Result: " + methodName, result);
  cv::imshow("Detected Point: " + methodName, img);
  cv::waitKey(0);
  cv::destroyAllWindows();
}