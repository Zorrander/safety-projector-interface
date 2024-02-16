#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Image.h>
#include <ros/header.h>

using namespace message_filters;
using namespace std;
static const std::string OPENCV_WINDOW = "Image window";
static const std::string SAFETY_WINDOW = "Safety line";

class SafetyDM
{
  private:
      ros::NodeHandle nh_;
      image_transport::ImageTransport it_;
      image_transport::Publisher pub_safety;
      image_transport::Publisher pub_safety_colored;
      image_transport::Publisher pub_safety_inside;
      message_filters::Subscriber<geometry_msgs::TransformStamped> base_link;
      message_filters::Subscriber<geometry_msgs::TransformStamped> link_4;
      message_filters::Subscriber<geometry_msgs::TransformStamped> link_5;
      message_filters::Subscriber<geometry_msgs::TransformStamped> tool_0;
      message_filters::Subscriber<geometry_msgs::TransformStamped> flange;
      typedef sync_policies::ApproximateTime<geometry_msgs::TransformStamped, geometry_msgs::TransformStamped, geometry_msgs::TransformStamped, geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> MySyncPolicy;
      typedef Synchronizer<MySyncPolicy> Sync;
      boost::shared_ptr<Sync> sync;
      image_transport::Subscriber dm_sub_;
      std::vector<geometry_msgs::Point> l_points;
      double ax;
      double bx;
      double ay;
      double by;
      double az;
      double bz;

  public:
      SafetyDM():
      it_(nh_)
      {
         base_link.subscribe(nh_, "/coords/base_link", 1);
         link_4.subscribe(nh_, "/coords/link_4", 1);
         link_5.subscribe(nh_, "/coords/link_5", 1);
         tool_0.subscribe(nh_, "/coords/tool0", 1);
         flange.subscribe(nh_, "/coords/flange", 1);
         sync.reset(new Sync(MySyncPolicy(10), base_link, link_4, link_5, tool_0, flange));      
         sync->registerCallback(boost::bind(&SafetyDM::callbackJoints3D, this, _1, _2, _3, _4, _5));
         pub_safety = it_.advertise("/safety_line/line_dm", 1);
         pub_safety_colored = it_.advertise("/safety_line/line_proj", 1);
         pub_safety_inside = it_.advertise("/safety_line/inside_zone", 1);
         dm_sub_ = it_.subscribe("/detection/depth_map", 1,&SafetyDM::callbackDepthMap, this);
         l_points.resize(0);
         cv::namedWindow(OPENCV_WINDOW,cv::WINDOW_NORMAL);
         readParamsDM();
      }

      ~SafetyDM()
      {
         cv::destroyWindow(OPENCV_WINDOW);
         cv::destroyWindow(SAFETY_WINDOW);
      }

      void callbackJoints3D(geometry_msgs::TransformStampedConstPtr ts_bl, geometry_msgs::TransformStampedConstPtr ts_l4, geometry_msgs::TransformStampedConstPtr ts_l5, geometry_msgs::TransformStampedConstPtr ts_tool, geometry_msgs::TransformStampedConstPtr ts_flange)
      {
         cv::Mat cv_image = cv::Mat::zeros(1024,1024,CV_8UC1);
         cv::Mat sf_line = cv::Mat::zeros(1024,1024,CV_8UC1);
         cv::Mat sf_line_inside = cv::Mat::zeros(1024,1024,CV_8UC1);
         cv::Mat sf_line_colored = cv::Mat::zeros(1024,1024,CV_8UC3);
         cv::Mat res_contours;
         int pixel_pos_x;
         int pixel_pos_y;
         l_points.resize(0);
         geometry_msgs::Point bl;
         geometry_msgs::Point l4;
         geometry_msgs::Point l5;
         geometry_msgs::Point tool;
         geometry_msgs::Point flg;
         bl.x = ts_bl->transform.translation.x;
         bl.y = ts_bl->transform.translation.y;
         bl.z = ts_bl->transform.translation.z;
         l4.x = ts_l4->transform.translation.x;
         l4.y = ts_l4->transform.translation.y;
         l4.z = ts_l4->transform.translation.z;
         l5.x = ts_l5->transform.translation.x;
         l5.y = ts_l5->transform.translation.y;
         l5.z = ts_l5->transform.translation.z;
         tool.x = ts_tool->transform.translation.x;
         tool.y = ts_tool->transform.translation.y;
         tool.z = ts_tool->transform.translation.z;
         flg.x = ts_flange->transform.translation.x;
         flg.y = ts_flange->transform.translation.y;
         flg.z = ts_flange->transform.translation.z;
         l_points.push_back(bl);
         l_points.push_back(l4);
         l_points.push_back(l5);
         l_points.push_back(tool);
         l_points.push_back(flg);

         for(int i = 0; i < l_points.size(); i++)
         {
            double x = l_points[i].x * 1000;
            double y = l_points[i].y * 1000;
            pixel_pos_x = (int) (ax * x + bx);
            pixel_pos_y = (int) (ay * y + by);
            cv::Point center(pixel_pos_x, pixel_pos_y);
            circle(cv_image, center,100, cv::Scalar(255, 255, 0), 1);
         }
         std::vector<std::vector<cv::Point>> contours;
         std::vector<cv::Vec4i> hierarchy;
         cv::findContours(cv_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
         std::vector<std::vector<cv::Point> >hull( contours.size() );
         for( size_t i = 0; i < contours.size(); i++ )
         {
            convexHull( contours[i], hull[i] );
         }
         drawContours(sf_line, hull, 0, cv::Scalar(255, 255, 255),5);
         drawContours(sf_line_colored, hull, 0, cv::Scalar(0, 0, 255),5);
         drawContours(sf_line_inside, hull, 0, cv::Scalar(255, 255, 255),cv::FILLED);

         sensor_msgs::ImagePtr msg_dm;
         msg_dm = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, sf_line).toImageMsg();
         pub_safety.publish(msg_dm);
         sensor_msgs::ImagePtr msg_col;
         msg_col = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC3, sf_line_colored).toImageMsg();
         pub_safety_colored.publish(msg_col);
         sensor_msgs::ImagePtr msg_inside;
         msg_inside = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, sf_line_inside).toImageMsg();
         pub_safety_inside.publish(msg_inside);
         cv::imshow(OPENCV_WINDOW, sf_line_inside);
         cv::waitKey(1);
      }

      void callbackDepthMap(const sensor_msgs::ImageConstPtr& msg_dm)
      {
         cv::Mat cv_image = cv::Mat::zeros(1024,1024,CV_8UC1);
         cv::Mat sf_line = cv::Mat::zeros(1024,1024,CV_8UC1);
         cv::Mat res_contours;
         int pixel_pos_x;
         int pixel_pos_y;
         cv_bridge::CvImagePtr cv_ptr;
         try
         {
            cv_ptr = cv_bridge::toCvCopy(msg_dm, sensor_msgs::image_encodings::TYPE_32FC1);
            
            for(int i = 0; i < l_points.size(); i++)
            {
               double x = l_points[i].x * 1000;
               double y = l_points[i].y * 1000;
               pixel_pos_x = (int) (ax * x + bx);
               pixel_pos_y = (int) (ay * y + by);
               cv::Point center(pixel_pos_x, pixel_pos_y);
               circle(cv_image, center,100, cv::Scalar(255, 255, 0), 1);
            }
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(cv_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            std::vector<std::vector<cv::Point> >hull( contours.size() );
            for( size_t i = 0; i < contours.size(); i++ )
            {
               convexHull( contours[i], hull[i] );
            }
            drawContours( cv_ptr->image, hull, 0, cv::Scalar(255, 255, 0));
         }
         catch (cv_bridge::Exception& e)
         {
               ROS_ERROR("cv_bridge exception: %s", e.what());
               return;
         }
         //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
         //cv::waitKey(1);
      }

      void readParamsDM()
      {
         std::string line;
         std::ifstream icpfile("/home/altair/odin/src/pcl_fusion/calibration/params.txt");
         if(icpfile.is_open())
         {
         getline(icpfile,line);
         ax = std::stod(line); 
         getline(icpfile,line);
         bx = std::stod(line);
         getline(icpfile,line);
         ay = std::stod(line);
         getline(icpfile,line);
         by = std::stod(line);
         getline(icpfile,line);
         az = std::stod(line);
         getline(icpfile,line);
         bz = std::stod(line);
         icpfile.close();
         }
      }

}; 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "safety_line");
  SafetyDM sf_dm;
  ros::spin();

  return 0;
}