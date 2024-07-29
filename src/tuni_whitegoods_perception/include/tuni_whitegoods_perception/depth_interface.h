#ifndef DepthInterface_H
#define DepthInterface_H


#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <unity_msgs/InterfacePOI.h>
#include <unity_msgs/poiPCL.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
//#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <ros/header.h>
#include <visualization_msgs/Marker.h>



using namespace std;

class DepthInterface
{
  private:
    typedef image_transport::SubscriberFilter ImageSubscriber;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    image_transport::ImageTransport it_;

    ros::Subscriber sub_scene;
    ros::Subscriber sub_hand_poi;
    ros::Subscriber sub_poi;
    ros::Subscriber img_callback;
    ros::Publisher pub_poi;
    ros::Publisher pub_poi_pcl;
    ros::Publisher pub_hand_tracker_dm;

    image_transport::Subscriber dm_sub_;

    ros::Publisher vis_pub;

    bool depth_images;
    unity_msgs::InterfacePOI list_points;
    unity_msgs::poiPCL list_points_hand;
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;

    cv::Mat cv_viz_depth;
    image_transport::Publisher pub_full_scene;
    void depthSceneCallback(const sensor_msgs::ImageConstPtr& depth_msg);

    ros::ServiceClient client;
    ros::ServiceClient client_pixel_to_3D;
  public:
    DepthInterface(ros::NodeHandle* nh);

    //get the rgb_to_depth and depth images
    void callbackRGBDepth(const sensor_msgs::ImageConstPtr& rgbdepth_msg);
    
    //callback that transform RGB point of the hands to their coordinates in the depthmap
    void handTrackerCallback(const unity_msgs::poiPCLConstPtr& msg);

    //get depthmap parameters
    void readParamsFile();

    bool tf_in;

    ros::NodeHandle* nh_;

    string tf_robot_frame;
    string tf_depth_frame;
    cv::Mat cv_depth_rgb;
    cv::Mat cv_depth;
};

#endif