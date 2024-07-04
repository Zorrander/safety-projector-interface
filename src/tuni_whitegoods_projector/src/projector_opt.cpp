/*
Class that project everything. An instance of this class is launched for each projector connected.
It basically receives a list of projection that contains everything to display (smart interface, borders...) 
each projection is an image with a transform to display them. 
For a system with only one camera and projector, it only need to run once.
*/

#include <ros/ros.h>
#include <iostream>
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
  //image_transport::ImageTransport it_;
  cv_bridge::CvImagePtr cv_ptr;
  ros::Subscriber transform_sub;
  int shift;
  int id_device;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_proj_img;

public:
  Projector():
  it_(nh_)
  {
    transform_sub = nh_.subscribe("/list_dp", 1, &Projector::transformProject,this);
    //ros param to get id of device (videoprojector) and how much to shift the screen. Since there is one projector by computer, shere is no need for shift anymore for HRC.
    ros::param::get("shiftX", shift);
    ros::param::get("id", id_device);
    cv::namedWindow(OPENCV_WINDOW,cv::WINDOW_NORMAL);
    cout << "Shift value: " << shift << endl;
    cv::moveWindow(OPENCV_WINDOW,shift, 0);
    cv::setWindowProperty(OPENCV_WINDOW, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    pub_proj_img = it_.advertise("/odin/visualization/projections", 1);
  }

  ~Projector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void transformProject(const unity_msgs::ListDataProj::ConstPtr& msg)
  {
    cv::Mat sum_img(1080,1920,CV_8UC3,cv::Scalar(0,0,0));
    cv::Mat img_transformed(1080,1920,CV_8UC3,cv::Scalar(0,0,0));
    cv::Mat translated_image(1080,1920,CV_8UC3,cv::Scalar(0,0,0));
    bool suc = false;
    for(int i = 0; i < msg->list_proj.size(); i++)
    {
      if(msg->list_proj[i].id == id_device)
      {
        suc = true;
        try
        {
          pub_proj_img.publish(msg->list_proj[i].img);

          cv_ptr = cv_bridge::toCvCopy(msg->list_proj[i].img, sensor_msgs::image_encodings::BGR8);
          cv::Mat hom = getMatrix(msg->list_proj[i].transform);
          cv::warpPerspective(cv_ptr->image,img_transformed,hom,sum_img.size());
          // get tx and ty values for translation
          float tx = float(0);
          float ty = float(0);
          // create the translation matrix using tx and ty
          float warp_values[] = { 1.0, 0.0, tx, 0.0, 1.0, ty };
          cv::Mat translation_matrix(2, 3, CV_32F, warp_values);
          // save the resulting image in translated_image matrix
          // apply affine transformation to the original image using the translation matrix
          warpAffine(img_transformed, translated_image, translation_matrix, img_transformed.size());
          sum_img = sum_img.clone() + translated_image.clone();
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
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

  cv::Mat getMatrix(const std::vector<double> mat)
  {
    double transform[3][3];
    cv::Mat cv_transform = cv::Mat(3,3,CV_32FC1);
    int k = 0;
    for(int i = 0; i < 3; i++)
    {
      for(int j = 0; j < 3; j++)
      {
        transform[i][j] = mat[k];
        cv_transform.at<float>(i,j) = mat[k];
        k++;
      }
    }
    
    return cv_transform;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "projector_optimized_one");
  Projector ic;
  ros::spin();
  return 0;
}