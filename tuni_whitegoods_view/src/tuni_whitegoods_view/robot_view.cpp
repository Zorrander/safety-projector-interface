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
    tableTopLeftCornerPt.x = 0.5769190341781458; 
    tableTopLeftCornerPt.y = 2.271676229695987;
    tableTopLeftCornerPt.z = -0.41678163128446966;


    geometry_msgs::Point tableTopRightCornerPt;
    tableTopRightCornerPt.x = 0.8398147036126069; 
    tableTopRightCornerPt.y = 2.003771048623398;
    tableTopRightCornerPt.z = -0.3878336824295425;

    geometry_msgs::Point tableBottomRightCornerPt;
    tableBottomRightCornerPt.x = 0.49672333993327816; 
    tableBottomRightCornerPt.y = 1.558380452870774;
    tableBottomRightCornerPt.z = -0.38744156965179033;

    geometry_msgs::Point tableBottomLeftCornerPt;
    tableBottomLeftCornerPt.x =  0.25405491899444177; 
    tableBottomLeftCornerPt.y = 1.791176063961366;
    tableBottomLeftCornerPt.z = -0.40378966384045234;
    
    table_points.push_back(tableTopLeftCornerPt); 
    table_points.push_back(tableTopRightCornerPt); 
    table_points.push_back(tableBottomRightCornerPt); 
    table_points.push_back(tableBottomLeftCornerPt); 

    createRvizMarker(table_points);

    std::vector<geometry_msgs::Point> shelf_points;
    geometry_msgs::Point shelfTopLeftCornerPt;
    shelfTopLeftCornerPt.x = 0.2459696255815204; 
    shelfTopLeftCornerPt.y = 1.0416911823654855;
    shelfTopLeftCornerPt.z = 0.1335070435087502;



    geometry_msgs::Point shelfTopRightCornerPt;
    shelfTopRightCornerPt.x = 1.1326225002348123; 
    shelfTopRightCornerPt.y = 0.21348169082051777;
    shelfTopRightCornerPt.z = 0.16334966343338553;



    geometry_msgs::Point shelfBottomRightCornerPt;
    shelfBottomRightCornerPt.x = 0.8757711748449375; 
    shelfBottomRightCornerPt.y = 0.030293135651739855;
    shelfBottomRightCornerPt.z = 0.15230856567818551;


    geometry_msgs::Point shelfBottomLeftCornerPt;
    shelfBottomLeftCornerPt.x = 0.05256043965352292; 
    shelfBottomLeftCornerPt.y = 0.7938490707142187;
    shelfBottomLeftCornerPt.z = 0.12675945620055273;

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
