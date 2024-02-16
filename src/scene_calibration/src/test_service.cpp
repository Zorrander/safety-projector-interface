#include <ros/ros.h>
#include <scene_calibration/PointsDepthMap.h>
#include <cstdlib>
#include <geometry_msgs/Point.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "points_client");
    ros::NodeHandle nh;
    ros::ServiceClient spawnClient = nh.serviceClient<scene_calibration::PointsDepthMap>("server_rgb_points");
    scene_calibration::PointsDepthMap::Request req;
    scene_calibration::PointsDepthMap::Response resp;
    geometry_msgs::Point p;
    p.x = 611.0;
    p.y = 150.0;
    req.poi_rgb.push_back(p);

    ros::service::waitForService("server_rgb_points", ros::Duration(5));
    bool success = spawnClient.call(req,resp);

    if(success)
    {
       ROS_INFO_STREAM("Spawned a turtle named "
                       << resp.poi_depth_map[0]);
    }
    else
    {
       ROS_ERROR_STREAM("Failed to spawn.");
    }
  

  return 0;
}