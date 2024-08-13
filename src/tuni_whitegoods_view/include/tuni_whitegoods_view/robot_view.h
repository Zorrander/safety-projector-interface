#ifndef RobotView_H
#define RobotView_H

#include <ros/ros.h>
#include <tuni_whitegoods_msgs/RobotViewElement.h>


class RobotView
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber marker_sub;
  ros::Publisher vis_pub;
  
public:
    RobotView();
    void createRvizMarker(const tuni_whitegoods_msgs::RobotViewElement::ConstPtr& msg);
};

#endif