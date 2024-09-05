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
  cv::Matx33d border_hom, button_hom;
  cv::Mat sum_img, button_img, border_img;

public:
    Projector();
    ~Projector();

    void init() override;
    void updateButtons(std::vector<std::shared_ptr<Button>> buttons) override;
    void updateBorders(std::vector<std::shared_ptr<StaticBorder>> borders) override;
    void updateHands(std::vector<std::shared_ptr<Hand>> hands) override;
};

#endif