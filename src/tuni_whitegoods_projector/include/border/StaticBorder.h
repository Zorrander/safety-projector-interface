#ifndef STATIC_BORDER_H
#define STATIC_BORDER_H

#include <ros/ros.h>
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
#include <cmath>
#include <string>
#include <std_msgs/ColorRGBA.h>

using namespace message_filters;
using namespace std;
static const std::string OPENCV_WIN = "Image window";

struct BorderStatus{
   std::string id_border;
   int status;
}  ;

class StaticBorder
{
   public:
      StaticBorder(std::string r_id, std::string z, int pos_row, int pos_col, geometry_msgs::PolygonStamped bord, std::string b_topic, std_msgs::ColorRGBA b_color, bool filling, int thic, ros::Duration life, bool track);

      float getBorderDiagonal();

      cv::Mat drawMask();
      std::string getZone() const; 
      std::string getId();
      int getCol();
      int getRow();
      bool getTracking();
      cv::Mat drawBorder();
      geometry_msgs::Point getCenter();
      void changeBorderColor(std_msgs::ColorRGBA& col);
      void changeThickness(int thic);
      std_msgs::ColorRGBA getColor();
      bool left_hand_crossed;
      bool right_hand_crossed;
      bool booked;
      bool occupied;
      void book();
      void release();
      geometry_msgs::PolygonStamped border_robot_space;
      cv::Point top_left_cam_point;
      cv::Point bottom_right_cam_point;

   private:
      
      
      cv::Mat border_mask;
      int position_row;
      int position_col;

      std_msgs::ColorRGBA border_color;
      bool is_filled;
      int thickness;
      ros::Duration lifetime;
      bool track_violations;
      std::string request_id;
      std::string zone;
      cv::Mat sf_line_colored;

};
#endif