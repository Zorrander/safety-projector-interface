/*
Class that project everything. An instance of this class is launched for each projector connected.
It basically receives a list of projection that contains everything to display (smart interface, borders...) 
each projection is an image with a transform to display them. 
For a system with only one camera and projector, it only need to run once.
*/

#include <ros/ros.h>


using namespace std;



class RobotView
{
  ros::NodeHandle nh_;
  ros::Subscriber marker_sub;

public:
  RobotView()
  {
    marker_sub = nh_.subscribe("/robot_frame_element", 1, &RobotView::createRvizMarker, this);
  }

  ~RobotView()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }


  void createRvizMarker(const unity_msgs::ListDataProj::ConstPtr& msg)
  {

   // Create Rviz marker 
   visualization_msgs::Marker marker;
   marker.header.frame_id = "base";
   marker.header.stamp = ros::Time::now();
   marker.id = borders.size()+10;
   marker.type = visualization_msgs::Marker::LINE_STRIP;
   marker.action = visualization_msgs::Marker::ADD;
   marker.scale.x = 0.01;  // Line width
   marker.color.r = 0.0;
   marker.color.g = 1.0;
   marker.color.b = 0.0;
   marker.color.a = 1.0;

   marker.points.push_back(topLeftCornerPt);
   marker.points.push_back(topRightCornerPt);
   marker.points.push_back(bottomRightCornerPt);
   marker.points.push_back(bottomLeftCornerPt);
   marker.points.push_back(topLeftCornerPt);

   vis_pub.publish( marker );
    
  }

};


