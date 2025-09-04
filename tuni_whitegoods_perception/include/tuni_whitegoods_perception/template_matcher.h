#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class TemplateMatcher {
 public:
  TemplateMatcher(const std::string& templatePath);
  bool scan(cv::Mat image, cv::Rect roi);

 private:
  cv::Mat sourceImage;
  cv::Mat templateImage0;
  cv::Mat templateImage1;
  cv::Mat templateImage2;
  cv::Mat templateImage3;
  cv::Mat templateImage4;
  cv::Mat templateImage5;
  cv::Mat templateImage6;
  cv::Mat templateImage7;

  std::vector<cv::Mat> templates;

  std::vector<int> methods;
  std::string imagePath;
  std::string templatePath;
  bool scan_result;

  cv::Mat centerCropToMatchTemplate(const cv::Mat& input, const cv::Mat& templ);
  void drawResult(const cv::Mat& img, const cv::Mat& result,
                  const cv::Point& topLeft, const std::string& methodName);
};