/*
Class that project everything. An instance of this class is launched for each projector connected.
It basically receives a list of projection that contains everything to display (smart interface, borders...) 
each projection is an image with a transform to display them. 
For a system with only one camera and projector, it only need to run once.
*/

#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Float64.h"
#include <unity_msgs/DataProj.h>
#include <unity_msgs/Projection.h>
#include <unity_msgs/ListProjection.h>
#include <unity_msgs/ListDataProj.h>

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class Projector
{
  ros::NodeHandle nh_;
  cv_bridge::CvImagePtr cv_ptr;
  ros::Subscriber transform_sub;
  int shift;
  cv::Matx33d hom;

public:
  Projector()
  {
    //ros param to get id of device (videoprojector) and how much to shift the screen. Since there is one projector by computer, shere is no need for shift anymore for HRC.
    ros::param::get("shiftX", shift);
    cv::namedWindow(OPENCV_WINDOW,cv::WINDOW_NORMAL);
    cv::moveWindow(OPENCV_WINDOW,shift, 0);
    cv::setWindowProperty(OPENCV_WINDOW, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    transform_sub = nh_.subscribe("/list_dp", 1, &Projector::transformProject,this);

    hom = cv::Matx33d(-2.15507712e+00,  1.91967042e-01,  2.86695078e+03, 
                      5.92436261e-03, -2.16676604e+00,  1.75534894e+03, 
                      1.69314309e-05,  2.45548501e-04,  1.00000000e+00);

  }

  ~Projector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }


  void transformProject(const unity_msgs::ListDataProj::ConstPtr& msg)
  {

    cv::Mat sum_img(1080,1920,CV_8UC3,cv::Scalar(0,0,0));
    cv::Mat img_transformed(1080,1920,CV_8UC3,cv::Scalar(0,0,0));

    bool suc = false;
    for(int i = 0; i < msg->list_proj.size(); i++)
    {
        suc = true;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg->list_proj[i].img, sensor_msgs::image_encodings::BGR8);
          cv::warpPerspective(cv_ptr->image, img_transformed, hom, sum_img.size());
          sum_img = sum_img.clone() + img_transformed.clone();    
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
    }
    
    // Update GUI Window
    if(suc == false)
    {
      sum_img.create(1080,1920,CV_8UC3);
    } 
    
    cv::imshow(OPENCV_WINDOW, sum_img);
    cv::waitKey(1);
    
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "projector_optimized_one");
  Projector ic;
  ros::spin();
  return 0;
}