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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
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

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

#include <actionlib/client/simple_action_client.h>

#include <unity_msgs/InterfacePOI.h>
#include <unity_msgs/ElementUI.h>
#include <integration/VirtualButtonEventArray.h>
#include <integration/SetVirtualButtonChangeColorAction.h>
#include <integration/SetVirtualButtonChangeColorGoal.h>
using namespace message_filters;
using namespace std;
static const std::string OPENCV_TEST = "Image window";

#define LEFT 0 
#define RIGHT 1

struct Projection {
   std::string zone;
   cv::Mat img;
   cv::Mat mask;
};

struct BorderContentStatus {
   std::string id;
   int status;
   int col;
   int row;
   geometry_msgs::Point center;
   float safety_distance;
   float side_distance;
};


struct Button {
   std::string id;
   geometry_msgs::Point center;
   bool left_hand_hover;
   bool right_hand_hover;
   bool operator==(const unity_msgs::ElementUI &other) const {
      return id == other.id;
   }
};


class StaticBorderManager
{
   public:
      StaticBorderManager(ros::NodeHandle *nh_, int size_rows, int size_cols, float sf_factor, bool adjacent, std_msgs::ColorRGBA status_booked, std_msgs::ColorRGBA status_free, std_msgs::ColorRGBA status_operator);
      
      void depthMapCallback(const sensor_msgs::ImageConstPtr& msg_dm);
      void handTrackingCallback(const unity_msgs::poiPCLConstPtr& msg);
      void raiseBorderViolation(std::vector<cv::KeyPoint>& keypoints, std_msgs::Header& header);
      std::vector<cv::KeyPoint> detectBorderCrossing(cv::Mat& mask);
      void addBorder(std::shared_ptr<StaticBorder> sb);
      void bookBorderRobot(std::string id);
      void bookBorderOperator(std::string id);
      void releaseRobotBorder(std::string id, int status);
      void releaseOperatorBorder(std::string id, int status);
      void updateProjection();
      std::vector<std::string> getAdjacentBorders(int row, int col);

      bool readRawImage(cv::Mat& image, const std::string& filename);
 
      void publishBorder();

      void pointsOfInterestCb(const unity_msgs::InterfacePOIConstPtr& msg);

      void getMaskDetection(cv::Mat& image);
      bool isClusterInsideBorder(cv::Mat& image, BorderContentStatus bdr);
      bool getBordersService(integration::ListStaticBordersStatus::Request& req, integration::ListStaticBordersStatus::Response& res);
      void updateBorderStatus();
      void getObjectsDepth(cv::Mat& depth_within_borders, const cv::Mat& mask_detect);
      cv::Mat enhanceDepth(cv::Mat img, float thr);



   private:
      ros::NodeHandle nh;
      image_transport::ImageTransport it_;
      image_transport::Subscriber dm_sub_;
      ros::ServiceServer service_borders;
      ros::Subscriber sub_hand_detection;

      

      ros::Publisher pub_border_projection;
      ros::Publisher pub_border_violation;
      ros::Publisher pub_pose_violation;
      
      std::vector<std::shared_ptr<StaticBorder>> borders;
      std::vector<Projection> list_proj;
      std::vector<BorderContentStatus> borders_status;
      std::vector<BorderContentStatus> borders_booked;
      std::vector<BorderContentStatus> list_hand_violation;
      ros::Publisher vis_pub;
      ros::Publisher vis_hand_pub;

      cv::Mat safety_line_mask;
      cv::Mat baseline_dm;
      cv::Mat depth_map;

      std::vector<float> dist_buffers;
      bool border_crossed; 
      bool button_pressed; 
      
      int s_rows;
      int s_cols;
      float safety_factor;
      bool redraw;
      bool adj;
      std_msgs::ColorRGBA stat_booked;
      std_msgs::ColorRGBA stat_free;
      std_msgs::ColorRGBA stat_operator;
      string calibration_folder;
      string name_bl;
      cv::Mat mask_detect;

      // tests
      ros::Publisher pub_event;
      ros::Subscriber sub_poi;
      std::vector<Button> buttons;
      actionlib::SimpleActionClient<integration::SetVirtualButtonChangeColorAction> client_button_color;
      ros::Subscriber depth_sub;
      cv::Mat cv_depth;
      void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg);

      std::vector<ros::Publisher> pubs_border_polygon;

      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener* tfListener;

      image_transport::Publisher object_detection_pub;
      tf2::Transform tf2_transform;
      cv::Rect roi_rect;
      // ros::Publisher pub_borders_vacancy;
      // cv::Mat res_dm;
      // cv::Point loc;
      // bool change_color;
      // cv::Mat sf_line;
      // bool first_baseline;
      // bool first_init;
      //float threshold_depth_inf;
      //float highest_depth;
      //std::vector<float> minmax_values;
      // bool crossed;
};
#endif