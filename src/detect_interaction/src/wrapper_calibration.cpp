#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <scene_calibration/PointsDepthMap.h>
#include <unity_msgs/InterfacePOI.h>
#include <unity_msgs/ElementUI.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <unity_msgs/poiPCL.h>

using namespace sensor_msgs;
using namespace message_filters;

class WrapperCalibration
{
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub;
        image_transport::Subscriber dm_sub_;
        ros::Subscriber sub_cam_info;
        ros::Subscriber sub_poi_pcl;
        ros::ServiceClient spawnClient;
        
    
  public:
    WrapperCalibration()
    {
      sub_poi_pcl = nh_.subscribe("/calibration/pts", 1, &WrapperCalibration::poiPCLCb,this);
      pub = nh_.advertise<unity_msgs::poiPCL> ("/calibration/calibrated_points", 1);
      spawnClient = nh_.serviceClient<scene_calibration::PointsDepthMap>("server_rgb_points");
    }
    ~WrapperCalibration()
    {
      
    }

    void poiPCLCb(const unity_msgs::poiPCLConstPtr& msg)
    {
        
        unity_msgs::poiPCL calibrated;
        scene_calibration::PointsDepthMap::Request req;
        scene_calibration::PointsDepthMap::Response resp;
        for(int i = 0; i < msg->pts.size(); i++)
        {
            req.poi_rgb.push_back(msg->pts[i]);
        }
        ros::service::waitForService("server_rgb_points", ros::Duration(5));
        bool success = spawnClient.call(req,resp);
        if(success)
        {
          
          for(int i = 0; i < resp.poi_depth_map.size(); i++)
          {
            geometry_msgs::Point p;
            p.x = resp.poi_depth_map[i].x;
            p.y = resp.poi_depth_map[i].y;
            p.z = resp.poi_depth_map[i].z;
            std::cout<<p.x<<"\n";
            std::cout<<p.y<<"\n";
            std::cout<<p.z<<"\n";
            calibrated.pts.push_back(p);
          } 
        }
        else
        {
          ROS_ERROR_STREAM("Request Failed.");
        }
        pub.publish(calibrated);
    }  
}; 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_interaction");
  WrapperCalibration cal;
  ros::spin();
  return 0;
}