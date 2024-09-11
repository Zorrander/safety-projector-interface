#ifndef Projector_H
#define Projector_H

#include "tuni_whitegoods_view/view.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tuni_whitegoods_msgs/Projection.h>

class Projector : public View
{
private:
  cv_bridge::CvImagePtr cv_ptr;
  int shift;

  cv::Mat sum_img, button_img, border_img;

  cv::Matx33d border_homography, button_homography;
  std::vector<double> border_homography_array, button_homography_array;
  std::vector<int> projector_resolution;
public:
    Projector(ros::NodeHandle *nh);
    ~Projector();

    void init() override;
    void updateButtons(const std::vector<std::shared_ptr<Button>>& buttons) override;
    void updateBorders(const std::vector<std::shared_ptr<StaticBorder>>& borders) override;
    void updateHands(const std::vector<std::shared_ptr<Hand>>& hands) override;
};

#endif