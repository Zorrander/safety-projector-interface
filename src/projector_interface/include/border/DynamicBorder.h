#ifndef DYNAMIC_BORDER_H
#define DYNAMIC_BORDER_H

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include "border/Border.h"
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
#include <std_msgs/Header.h>
#include "unity_msgs/BorderProj.h"
#include <integration/SafetyBorderViolation.h>
#include <string>

using namespace message_filters;
using namespace std;
static const std::string OPENCV_WINDOW = "Image window";

class DynamicBorder : public Border
{
   public:
      DynamicBorder(ros::NodeHandle *nh_, std::string r_id, std::string z, std::string b_topic, std_msgs::ColorRGBA b_color, bool filling, int thic, ros::Duration life, bool track);
      DynamicBorder(const DynamicBorder &cpctr);
      void drawBorder(std_msgs::Header head);
      void callbackJoints3D(geometry_msgs::TransformStampedConstPtr ts_bl, geometry_msgs::TransformStampedConstPtr ts_l4, geometry_msgs::TransformStampedConstPtr ts_l5, geometry_msgs::TransformStampedConstPtr ts_tool, geometry_msgs::TransformStampedConstPtr ts_flange);
      void depthMapCallback(const sensor_msgs::ImageConstPtr& msg_dm);
      void lineCallback(const sensor_msgs::ImageConstPtr& msg_dm);
      void transformToRobotSpace(geometry_msgs::PolygonStamped cv_poly);
      void publishHulls();
      bool detectCluster(cv::Mat img);
      void echo();
   private:
      ros::NodeHandle nh;
      image_transport::ImageTransport it_;
      message_filters::Subscriber<geometry_msgs::TransformStamped> base_link;
      message_filters::Subscriber<geometry_msgs::TransformStamped> link_4;
      message_filters::Subscriber<geometry_msgs::TransformStamped> link_5;
      message_filters::Subscriber<geometry_msgs::TransformStamped> tool_0;
      message_filters::Subscriber<geometry_msgs::TransformStamped> flange;
      typedef sync_policies::ApproximateTime<geometry_msgs::TransformStamped, geometry_msgs::TransformStamped, geometry_msgs::TransformStamped, geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> MySyncPolicy;
      typedef Synchronizer<MySyncPolicy> Sync;
      boost::shared_ptr<Sync> sync;
      image_transport::Subscriber test_;
      std::vector<geometry_msgs::Point> l_points;
      std::vector<geometry_msgs::Point> prev_l_points;
      image_transport::Publisher pub_safety_colored;
      image_transport::Publisher pub_safety_inside;
      ros::Publisher pub_border_poly;
      ros::Publisher pub_pose_violation;
      ros::Publisher pub_border_projection;
      geometry_msgs::PolygonStamped border;
      std_msgs::Header header_border;
      cv::Mat cv_image;
      cv::Mat sf_line;
      bool first_baseline;
      bool got_mask;
      float threshold_depth_inf;
      geometry_msgs::PoseStamped test_pose;
      bool joints_moving;
      float threshold_joints;
      bool lock_zone;
};
#endif