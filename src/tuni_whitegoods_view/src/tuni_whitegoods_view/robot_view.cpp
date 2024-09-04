#include "tuni_whitegoods_view/robot_view.h"
#include <visualization_msgs/Marker.h>

using namespace std;

RobotView::RobotView() {
  vis_pub =
      nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  marker_counter = 1;
  ROS_INFO("RobotView running");
}

void RobotView::init() {
    std::vector<geometry_msgs::Point> table_points;
    geometry_msgs::Point tableTopLeftCornerPt;
    tableTopLeftCornerPt.x = 0.4842356167339994; 
    tableTopLeftCornerPt.y = 1.649077461104246;

    geometry_msgs::Point tableTopRightCornerPt;
    tableTopRightCornerPt.x = 0.7011640489770836; 
    tableTopRightCornerPt.y = 1.4475962357262455;

    geometry_msgs::Point tableBottomRightCornerPt;
    tableBottomRightCornerPt.x = 0.4689754590291647; 
    tableBottomRightCornerPt.y = 1.1961234770006834;

    geometry_msgs::Point tableBottomLeftCornerPt;
    tableBottomLeftCornerPt.x = 0.25204702678608054; 
    tableBottomLeftCornerPt.y = 1.397604702378684;

    table_points.push_back(tableTopLeftCornerPt); 
    table_points.push_back(tableTopRightCornerPt); 
    table_points.push_back(tableBottomRightCornerPt); 
    table_points.push_back(tableBottomLeftCornerPt); 

    createRvizMarker(table_points);

    std::vector<geometry_msgs::Point> shelf_points;
    geometry_msgs::Point shelfTopLeftCornerPt;
    shelfTopLeftCornerPt.x = 0.24327004565708776; 
    shelfTopLeftCornerPt.y = 1.041316690374685;

    geometry_msgs::Point shelfTopRightCornerPt;
    shelfTopRightCornerPt.x = 1.1344354828701788; 
    shelfTopRightCornerPt.y = 0.21361004792307275;

    geometry_msgs::Point shelfBottomRightCornerPt;
    shelfBottomRightCornerPt.x = 0.8962419947096597; 
    shelfBottomRightCornerPt.y = -0.04436633925265343;

    geometry_msgs::Point shelfBottomLeftCornerPt;
    shelfBottomLeftCornerPt.x = 0.00507655749656849; 
    shelfBottomLeftCornerPt.y = 0.7833403031989589;

    shelf_points.push_back(shelfTopLeftCornerPt); 
    shelf_points.push_back(shelfTopRightCornerPt); 
    shelf_points.push_back(shelfBottomRightCornerPt); 
    shelf_points.push_back(shelfBottomLeftCornerPt); 

    createRvizMarker(shelf_points);

    ROS_INFO("RobotView init");
}

void RobotView::updateButtons(std::vector<std::shared_ptr<Button>> buttons) {
  for (auto& button: buttons){
      std::vector<geometry_msgs::Point> points(1, button->center.position);
      createRvizMarker(points); 
  }
}

void RobotView::updateBorders(std::vector<std::shared_ptr<StaticBorder>> borders) {
  for (auto& border: borders){
      std::vector<geometry_msgs::Point> points;

      points.push_back(border->topLeftCornerPt);
      points.push_back(border->topRightCornerPt);
      points.push_back(border->bottomRightCornerPt);
      points.push_back(border->bottomLeftCornerPt);

      createRvizMarker(points); 
  }
}

void RobotView::updateHands(std::vector<std::shared_ptr<Hand>> hands) {
  for (auto& hand: hands){
      std::vector<geometry_msgs::Point> points(1, hand->robot_frame_position);
      createRvizMarker(points, 0); 
  }
}


void RobotView::createRvizMarker(std::vector<geometry_msgs::Point> points, int id) {
  // Create Rviz marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base";
  marker.header.stamp = ros::Time::now();
  marker.id = (id == 0) ? id : marker_counter;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.01; // Line width
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  if (points.size() == 1){
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position = points[0];
    marker.scale.x = 0.05; 
    marker.scale.y = 0.05;  
    marker.scale.z = 0.05;  
  } else {
    marker.type = visualization_msgs::Marker::LINE_STRIP;

    for (auto& point: points){
      marker.points.push_back(point);  
    }
    marker.points.push_back(points[0]); 
  }
  

 
  vis_pub.publish(marker);
  marker_counter+=1;
}
