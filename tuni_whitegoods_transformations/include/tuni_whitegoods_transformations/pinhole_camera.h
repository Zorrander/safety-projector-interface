#ifndef PinholeCamera_H
#define PinholeCamera_H


#include <opencv2/core/mat.hpp>


class PinholeCamera
{
  private:
    cv::Matx33d K_;

  public:
    PinholeCamera();
    double cx() const;
    double cy() const;
    double fx() const;
    double fy() const;

};

#endif