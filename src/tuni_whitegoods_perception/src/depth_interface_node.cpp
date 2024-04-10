#include "tuni_whitegoods_perception/depth_interface.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_depthmap");
  ros::NodeHandle nh;

  string d;
  string drgb;

  ros::param::get("topic_depth", d);
  ros::param::get("topic_depth_to_rgb", drgb);

  DepthInterface sp(&nh, drgb, d);
  ros::spin();

  return 0;
}