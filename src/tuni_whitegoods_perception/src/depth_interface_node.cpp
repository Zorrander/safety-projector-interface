#include "tuni_whitegoods_perception/depth_interface.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_depthmap");

  string d;
  string drgb;

  ros::param::get("topic_depth", d);
  ros::param::get("topic_depth_to_rgb", drgb);
  ros::NodeHandle nh;
  DepthInterface depthInterface(&nh, drgb, d);
  ROS_INFO("Depth Interface running.");
  ros::spin();

  return 0;
}

