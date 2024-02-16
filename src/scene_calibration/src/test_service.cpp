#include <ros/ros.h>
#include <scene_calibration/PointsDepthMap.h>
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <integration/ListStaticBordersStatus.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "border_status");
    ros::NodeHandle nh;
    ros::ServiceClient spawnClient = nh.serviceClient<integration::ListStaticBordersStatus>("server_borders");
    integration::ListStaticBordersStatus::Request req;
    integration::ListStaticBordersStatus::Response resp;

    ros::service::waitForService("server_borders", ros::Duration(5));
    bool success = spawnClient.call(req,resp);

    if(success)
    {
       for(int i = 0; i < resp.status_borders.size(); i++)
       {
         std::cout<<"border : "<<resp.status_borders[i].id<<" is : "<<resp.status_borders[i].status<<"\n";
       }
    }
    else
    {
       ROS_ERROR_STREAM("Failed to get answer");
    }
  

  return 0;
}