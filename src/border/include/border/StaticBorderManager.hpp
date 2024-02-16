#ifndef STATIC_BORDER_MANAGER_H
#define STATIC_BORDER_MANAGER_H

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include "border/StaticBorder.hpp"
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
#include "unity_msgs/LBorder.h"
#include "unity_msgs/BorderID.h"
#include "unity_msgs/LStaticBorder.h"

using namespace message_filters;
using namespace std;
static const std::string OPENCV_TEST = "Image window";

struct Projection {
   std::string id;
   std::string zone;
   cv::Mat img;
   cv::Mat mask;
};

struct BorderContent {
   std::string id;
   geometry_msgs::Point center;
   bool occupied;
   float baseline_depth;
   cv::Mat baseline;
};

class StaticBorderManager
{
   public:
      StaticBorderManager(ros::NodeHandle *nh_, StaticBorder sb);
      void drawBorder(string id_draw);
      void depthMapCallback(const sensor_msgs::ImageConstPtr& msg_dm);
      void addBorder(StaticBorder sb);
      float getHighestPixel(cv::Mat img);
      cv::Mat enhanceDepth(cv::Mat img, float thr);
      int getViolatedBorder();
      void publishBorder();
      void publishOccupancy();
      bool detectCluster(cv::Mat img, int kernel, int thr);
      bool detectClusterOpt(cv::Mat img, int kernel, int thr, int miny, int maxy);
      void detectContent( cv::Mat curr);
      std::vector<float> getMinMax();
      //std::vector<geometry_msgs::Pose> getAvailableBorder();
      
   private:
      ros::NodeHandle nh;
      image_transport::ImageTransport it_;
      image_transport::Subscriber dm_sub_;
      ros::Publisher pub_border_projection;
      ros::Publisher pub_border_violation;
      ros::Publisher pub_pose_violation;
      ros::Publisher pub_borders_vacancy;
      std::vector<StaticBorder> borders;
      std::vector<Projection> list_proj;
      std::vector<BorderContent> list_content;
      cv::Mat safety_line_mask;
      cv::Mat sf_line_inside;
      cv::Mat sf_line_colored;
      cv::Mat baseline_dm;
      cv::Mat sf_line;
      cv::Mat depth_map;
      cv::Mat res_dm;
      bool first_baseline;
      bool first_init;
      bool redraw;
      float threshold_depth_inf;
      float highest_depth;
      geometry_msgs::PoseStamped pose_location;
      std::vector<float> minmax_values;
      bool crossed;
      int index;
      string index_redraw;
      int previous_index;
      string prev_index_redraw;
      cv::Point loc;
      bool change_color;
};
#endif