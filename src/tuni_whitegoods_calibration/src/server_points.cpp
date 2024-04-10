/*
This code is exactly the same as the package depth_interface
The ony difference is to be used only for calibration because it's a service
it uses tf to get the transform camera to robot space
*/

#include "tuni_whitegoods_msgs/PointsDepthMap.h"
#include "tuni_whitegoods_perception/depth_interface.h"

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
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

static const std::string OPENCV_WINDOW = "Image window";

class ServerPoints : public DepthInterface
{
  private:
    ros::ServiceServer service_points;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    geometry_msgs::TransformStamped transformStamped;

    Eigen::Affine3d robot_space;

    std::vector<pcl::PointXYZ> pts_map;

  public:

    ServerPoints(ros::NodeHandle *nh, std::string drgb, std::string d): 
                  DepthInterface(nh, drgb, d),
                  tfListener(tfBuffer)
    {
      service_points = nh->advertiseService("server_rgb_points", &ServerPoints::getPointsService, this);
      pts_map.resize(0);
      
      robot_space = Eigen::Affine3d::Identity();
      cv::namedWindow(OPENCV_WINDOW,cv::WINDOW_NORMAL);
    }

    void listenTransform()
    {
      bool error_in = false;
      bool error_rgb = false;
      Eigen::Affine3d cmp = Eigen::Affine3d::Identity();
      ROS_INFO("Listen transform");
      //compare transformation matrix, much more robust than a boolean to test if the transform has been handled
      while(cmp.isApprox(robot_space))
      {
        try
        {
          transformStamped = tfBuffer.lookupTransform(tf_robot_frame, tf_depth_frame,ros::Time(0), ros::Duration(5));
        } 
        catch (tf2::TransformException &ex) 
        {
          error_in = false;
          ROS_WARN("%s", ex.what());
          ros::Duration(1.0).sleep();
        }

        {
          cout<<"transform base master\n";
          tf_in = true;
          robot_space = tf2::transformToEigen(transformStamped);
        }
      }
      cout<<"success transform base\n";
      tf_in = true;
    }


    bool getPointsService(tuni_whitegoods_msgs::PointsDepthMap::Request& req, tuni_whitegoods_msgs::PointsDepthMap::Response& res)
    {
      if(tf_in == false)
      {
        listenTransform();
      }
      ROS_INFO("transformed listen");
      pts_map.resize(0);
      
      for(int i = 0; i < req.poi_rgb.size(); i++)
      {
        
        pcl::PointXYZ p;
        p.x = req.poi_rgb[i].x;
        p.y = req.poi_rgb[i].y;
        p.z = req.poi_rgb[i].z;
        pcl::PointXYZ pixel_res = getDepthFromRGB(p);
        //generate a point XYZ and apply tf
        pcl::PointXYZ pt_depth = generatePointCloudPOI(pixel_res);
        pcl::PointXYZ final_pt = genPointDepthMap(pt_depth);
        pts_map.push_back(final_pt);
      }
      ROS_INFO("req poi");
      res.poi_depth_map.resize(0);
      for(int i = 0; i < pts_map.size(); i++)
      {
        geometry_msgs::Point p;
        p.x = pts_map[i].x;
        p.y = pts_map[i].y;
        p.z = pts_map[i].z;
        res.poi_depth_map.push_back(p);
      }
      ROS_INFO("pts_map listen");
      return true;
    }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_depthmap");
  ros::NodeHandle nh;

  std::string d;
  std::string drgb;
      
  ros::param::get("topic_depth", d);
  ros::param::get("topic_depth_to_rgb", drgb);
  ServerPoints sp(&nh, drgb, d);
  ROS_INFO("Ready to get some points...");
  ros::spin();

  return 0;
}