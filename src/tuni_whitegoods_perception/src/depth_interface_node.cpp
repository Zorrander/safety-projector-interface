#include "tuni_whitegoods_perception/depth_interface.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_depthmap");
  string cam_name;
  string d;
  string drgb;
  ros::param::get("~camera_name", cam_name);
  ros::param::get("~topic_depth", d);
  ros::param::get("~topic_depth_to_rgb", drgb);
  ROS_INFO("Starting depth interface %s", ("/"+cam_name+drgb).c_str());
  ROS_INFO("Starting depth interface %s", ("/"+cam_name+d).c_str());
  ROS_INFO("Starting depth interface %s", cam_name.c_str());
  ros::NodeHandle nh;
  DepthInterface depthInterface(&nh, "/"+cam_name+drgb, "/"+cam_name+d, cam_name);
  ROS_INFO("Depth Interface running.");
  ros::spin();

  return 0;
}

