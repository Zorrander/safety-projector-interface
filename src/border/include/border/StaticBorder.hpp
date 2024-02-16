#ifndef STATIC_BORDER_H
#define STATIC_BORDER_H

#include <ros/ros.h>
#include "border/Border.hpp"
#include <std_msgs/Int64.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Header.h>
#include "unity_msgs/BorderProj.h"
#include <integration/SafetyBorderViolation.h>

using namespace message_filters;
using namespace std;
static const std::string OPENCV_WIN = "Image window";

class StaticBorder : public Border
{
   public:
      StaticBorder(std::string r_id, std::string z, geometry_msgs::PolygonStamped bord ,std::string b_topic, std_msgs::ColorRGBA b_color, bool filling, int thic, ros::Duration life, bool track);
      StaticBorder(const StaticBorder &cpctr);
      geometry_msgs::PolygonStamped getBorderCameraSpace();
      geometry_msgs::PolygonStamped getBorderRobotSpace();
      cv::Mat drawMask();
      std::string getZone();
      std::string getId();
      bool getTracking();
      cv::Mat drawBorder();
      geometry_msgs::Point getCenter();
      void changeBorderColor(std_msgs::ColorRGBA col);
      
   private:
      geometry_msgs::PolygonStamped border_robot_space;
      geometry_msgs::PolygonStamped border_camera_space;
      cv::Mat border_mask;
      
};
#endif