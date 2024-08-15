#include "tuni_whitegoods_view/robot_view.h"
#include <visualization_msgs/Marker.h>

using namespace std;

RobotView::RobotView()
{
  marker_sub = nh_.subscribe("/robot_frame_element", 1, &RobotView::createRvizMarker, this);
  vis_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  ROS_INFO("RobotView running");
}

void RobotView::init() {

}

void RobotView::update() {
	
}

void RobotView::createRvizMarker(const tuni_whitegoods_msgs::RobotViewElement::ConstPtr& msg)
{

 // Create Rviz marker 
 visualization_msgs::Marker marker;
 marker.header.frame_id = "base";
 marker.header.stamp = ros::Time::now();
 marker.id = msg->id;
 marker.type = visualization_msgs::Marker::LINE_STRIP;
 marker.action = visualization_msgs::Marker::ADD;
 marker.scale.x = 0.01;  // Line width
 marker.color.r = 0.0;
 marker.color.g = 1.0;
 marker.color.b = 0.0;
 marker.color.a = 1.0;

 // marker.points.push_back(topLeftCornerPt);
 // marker.points.push_back(topRightCornerPt);
 // .
 // .
 // .
 // marker.points.push_back(topLeftCornerPt);

 vis_pub.publish( marker );
  
}




