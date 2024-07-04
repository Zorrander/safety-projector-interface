#ifndef BORDER_H
#define BORDER_H

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/ColorRGBA.h>
#include <ros/duration.h>
#include <string>
//#include "integration/SafetyBorderViolation.h"

class Border
{
   public:
      //virtual void drawBorder(std_msgs::Header head) = 0;
      //virtual void depthMapCallback(const sensor_msgs::ImageConstPtr& msg_dm) = 0;
      void readParamsDepthMap();
      geometry_msgs::PolygonStamped transformToDepthMap(geometry_msgs::PolygonStamped border);
      geometry_msgs::Pose transformPtToDepthMap(double x, double y);
      geometry_msgs::Pose transformPtToRobotSpace(int px, int py);

   protected:
      image_transport::Subscriber dm_sub_;
      ros::Publisher pub_border_violation;   
      double ax;
      double bx;
      double ay;
      double by;
      double az;
      double bz;
      std::string request_id;
      std::string zone;
      std::string border_topic;
      std_msgs::ColorRGBA border_color;
      bool is_filled;
      int thickness;
      ros::Duration lifetime;
      bool track_violations; 
      cv::Mat safety_line_mask;
      cv::Mat sf_line_inside;
      cv::Mat sf_line_colored;
      cv::Mat baseline_dm;
      geometry_msgs::Pose location_violation;
      Border& operator=(const Border&);
};
#endif