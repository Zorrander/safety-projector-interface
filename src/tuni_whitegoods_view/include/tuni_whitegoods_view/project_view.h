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
  ros::NodeHandle nh_;
  cv_bridge::CvImagePtr cv_ptr;
  ros::Subscriber transform_sub;
  int shift;
  cv::Matx33d hom;
  cv::Mat sum_img;

public:
    Projector();
    ~Projector();

    void init() override;
    void updateButtons(std::vector<std::shared_ptr<Button>> buttons) override;
    void updateBorders(std::vector<std::shared_ptr<StaticBorder>> borders) override;
    void updateHands(std::vector<std::shared_ptr<Hand>> hands) override;
    
    void transformProject(const tuni_whitegoods_msgs::Projection::ConstPtr& msg);
};

#endif