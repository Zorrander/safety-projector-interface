#ifndef STATIC_BORDER_MANAGER_H
#define STATIC_BORDER_MANAGER_H

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/ColorRGBA.h>
#include "border/StaticBorder.h"
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
#include <integration/StaticBorderStatus.h>
#include <integration/ListStaticBordersStatus.h>
#include <unity_msgs/poiPCL.h>
#include <cmath>
#include <string>


using namespace message_filters;
using namespace std;
static const std::string OPENCV_TEST = "Image window";

struct Projection {
   std::string zone;
   cv::Mat img;
   cv::Mat mask;
};

struct BorderStatus{
   std::string id_border;
   int status;
}  ;

struct BorderContentStatus {
   std::string id;
   int status;
   int col;
   int row;
   geometry_msgs::Point center;
   float safety_distance;
   float side_distance;
};

class StaticBorderManager
{
   public:
      StaticBorderManager(ros::NodeHandle *nh_, int size_rows, int size_cols, float sf_factor, bool adjacent, std_msgs::ColorRGBA status_booked, std_msgs::ColorRGBA status_free, std_msgs::ColorRGBA status_operator);
      bool getBordersService(integration::ListStaticBordersStatus::Request& req, integration::ListStaticBordersStatus::Response& res);
      void depthMapCallback(const sensor_msgs::ImageConstPtr& msg_dm);
      void handTrackingCallback(const unity_msgs::poiPCLConstPtr& msg);
      void raiseBorderViolation(std::vector<cv::KeyPoint>& keypoints, std_msgs::Header& header);
      std::vector<cv::KeyPoint> detectBorderCrossing(cv::Mat& mask);
      void addBorder(StaticBorder sb);
      void bookBorderRobot(std::string id);
      void bookBorderOperator(std::string id);
      void releaseRobotBorder(std::string id, int status);
      void releaseOperatorBorder(std::string id, int status);
      void updateProjection();
      std::vector<std::string> getAdjacentBorders(int r, int c);
      void getObjectsBorders(cv::Mat& image);
      void getMaskDetection(cv::Mat& image);
      bool readRawImage(cv::Mat& image, const std::string& filename);
      void fillOccupancy(cv::Mat& image);
      bool isClusterInsideBorder(cv::Mat& image, BorderContentStatus bdr);
      cv::Mat enhanceDepth(cv::Mat img, float thr);
      void publishBorder();
      
   private:
      ros::NodeHandle nh;
      image_transport::ImageTransport it_;
      image_transport::Subscriber dm_sub_;
      ros::ServiceServer service_borders;
      ros::Subscriber sub_hand_detection;
      ros::Publisher pub_border_projection;
      ros::Publisher pub_border_violation;
      ros::Publisher pub_pose_violation;
      ros::Publisher pub_border_polygon;
      ros::Publisher pub_borders_vacancy;
      std::vector<StaticBorder> borders;
      std::vector<Projection> list_proj;
      std::vector<BorderContentStatus> borders_status;
      std::vector<BorderContentStatus> borders_booked;
      std::vector<BorderContentStatus> list_hand_violation;
      cv::Mat safety_line_mask;
      cv::Mat sf_line_inside;
      cv::Mat sf_line_colored;
      cv::Mat baseline_dm;
      cv::Mat sf_line;
      cv::Mat depth_map;
      cv::Mat res_dm;
      int s_rows;
      int s_cols;
      float safety_factor;
      bool adj;
      std_msgs::ColorRGBA stat_booked;
      std_msgs::ColorRGBA stat_free;
      std_msgs::ColorRGBA stat_operator;
      bool first_baseline;
      bool first_init;
      bool redraw;
      float threshold_depth_inf;
      float highest_depth;
      std::vector<float> minmax_values;
      bool crossed;
      int index;
      string index_redraw;
      int previous_index;
      string prev_index_redraw;
      string calibration_folder;
      string name_bl;
      cv::Point loc;
      bool change_color;
};
#endif