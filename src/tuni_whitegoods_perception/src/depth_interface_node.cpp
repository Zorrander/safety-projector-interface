#include "tuni_whitegoods_perception/depth_interface.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_depthmap");

  ros::NodeHandle nh;
  DepthInterface depthInterface(&nh);
  ROS_INFO("Depth Interface running.");
  ros::spin();

  return 0;
}

