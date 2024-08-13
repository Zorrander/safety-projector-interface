#include "tuni_whitegoods_view/project_view.h"

#include <sensor_msgs/image_encodings.h>


using namespace std;

static const std::string OPENCV_WINDOW = "Image window";


Projector::Projector()
{

  ros::param::get("shiftX", shift);
  cv::namedWindow(OPENCV_WINDOW,cv::WINDOW_NORMAL);
  cv::moveWindow(OPENCV_WINDOW,shift, 0);
  cv::setWindowProperty(OPENCV_WINDOW, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

  transform_sub = nh_.subscribe("/list_dp", 1, &Projector::transformProject,this);

  hom = cv::Matx33d(-2.15507712e+00,  1.91967042e-01,  2.86695078e+03, 
                    5.92436261e-03, -2.16676604e+00,  1.75534894e+03, 
                    1.69314309e-05,  2.45548501e-04,  1.00000000e+00);
  sum_img.create(1080, 1920, CV_8UC3);
  cv::imshow(OPENCV_WINDOW, sum_img);
  cv::waitKey(1);
}

Projector::~Projector()
{
  cv::destroyWindow(OPENCV_WINDOW);
}


void Projector::transformProject(const tuni_whitegoods_msgs::Projection::ConstPtr& msg)
{

  cv::Mat img_transformed(1080, 1920, CV_8UC3, cv::Scalar(0,0,0));

      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg->img, sensor_msgs::image_encodings::BGR8);
        cv::warpPerspective(cv_ptr->image, img_transformed, hom, sum_img.size());
        sum_img = sum_img.clone() + img_transformed.clone();    
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

  
  cv::imshow(OPENCV_WINDOW, sum_img);
  cv::waitKey(1);
  
}

