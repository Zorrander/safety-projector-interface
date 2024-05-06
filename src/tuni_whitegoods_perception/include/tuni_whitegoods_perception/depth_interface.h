#ifndef DepthInterface_H
#define DepthInterface_H


#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <fstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
#include <k4a/k4atypes.h>
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
#include <unity_msgs/AffineDepthMap.h>


using namespace std;

class DepthInterface
{
  private:
    typedef image_transport::SubscriberFilter ImageSubscriber;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    string calibration_folder;
    vector<double>  master_to_base;
    string file_recorded;

    image_transport::ImageTransport it_;

    ImageSubscriber depth_rgb_sub_;
    ImageSubscriber depth_sub_;

    ros::Subscriber sub_hand_poi;
    ros::Subscriber sub_poi;
    ros::Publisher pub_poi;
    ros::Publisher pub_poi_pcl;
    ros::Publisher pub_hand_tracker_dm;
    ros::Publisher depth_pub;
    image_transport::Subscriber dm_sub_;
    
    message_filters::Synchronizer< MySyncPolicy > sync;

    k4a::playback handle;
    k4a::calibration k4aCalibration;
    k4a::transformation k4aTransformation;

    geometry_msgs::Transform rgb_to_base;

    ros::Subscriber sub_cloud;
    ros::Publisher pub;
    string name_topic_pcl_tf, name_sub;
    string camera_name;

    int num_cam;
    bool depth_images;
    Eigen::Affine3d robot_space;
    Eigen::Matrix4d robot_space_mat;
    Eigen::Matrix4d icp_matrix;
    unity_msgs::InterfacePOI list_points;
    
    double ax;
    double bx;
    double ay;
    double by;
    double az;
    double bz;
    

  public:
    DepthInterface(ros::NodeHandle* nh, string name_drgb, string name_d, string cam_name);
    //get the camera robot transform
    void initTransformToBase();
    void initTransformToBaseTF();

    //get the rgb_to_depth and depth images
    void callbackRGBDepth(const sensor_msgs::ImageConstPtr& rgbdepth_msg, const sensor_msgs::ImageConstPtr& depth_msg);
    
    //callback that transform RGB point of the hands to their coordinates in the depthmap
    void handTrackerCallback(const unity_msgs::poiPCLConstPtr& msg);

    //get rgb points from interface and find them in the global depth map
    //void poiCallback(const unity_msgs::InterfacePOIConstPtr& msg);

   // get the depth value of the interface pixels given their location in the RGB space
    // pcl::PointXYZ getDepthFromRGB(cv::Mat depth_rgb, cv::Mat depth, pcl::PointXYZ p);

    // generate the 3D point(s) of the RGB coordinates we are interested in (hands, interface buttons)
    //pcl::PointXYZ generatePointCloudPOI(cv::Mat depth_image, pcl::PointXYZ pix);

    // fill point cloud with only the point we are interested in, then we apply transform to robot frame.
    //pcl::PointXYZ fillPointCloud(const k4a::image& pointcloud_image, sensor_msgs::PointCloud2Ptr& point_cloud);

    //Transform PointXYZ from point cloud to the depth map perspective
    //pcl::PointXYZ genPointDepthMap(pcl::PointXYZ point_d);


    //get depthmap parameters
    void readParamsFile();

    void alignToRobotBase(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    void readICPFromFile();

    bool tf_in;

    ros::NodeHandle* nh_;

    string tf_robot_frame;
    string tf_depth_frame;
    cv::Mat cv_depth_rgb;
    cv::Mat cv_depth;
};

#endif