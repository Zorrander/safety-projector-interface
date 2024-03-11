#include "border/Border.h"


/*void Border::drawBorder(std_msgs::Header head)
{

}

void Border::depthMapCallback(const sensor_msgs::ImageConstPtr& msg_dm)
{

}*/
//reads params for depthmap
void Border::readParamsDepthMap()
{
   ros::param::get("a2x", ax);
   ros::param::get("b2x", bx);
   ros::param::get("a2y", ay);
   ros::param::get("b2y", by);
   ros::param::get("a2z", az);
   ros::param::get("b2z", bz);
}
//tranform a Polygon from the robot space to the depthmap
geometry_msgs::PolygonStamped Border::transformToDepthMap(geometry_msgs::PolygonStamped border)
{
   readParamsDepthMap();
   geometry_msgs::PolygonStamped tmp;
   tmp.polygon.points.clear();
   for(geometry_msgs::Point32 i : border.polygon.points)
   {
      float pixel_pos_x;
      float pixel_pos_y;
      float pixel_pos_z;
      float x;
      float y;
      float z;
      x = i.x * 1000;
      y = i.y * 1000;
      z = i.z * 1000;
      pixel_pos_x = (int) (ax * x + bx);
      pixel_pos_y = (int) (ay * y + by);
      pixel_pos_z = (int) (az * z + bz);
      geometry_msgs::Point32 p;
      p.x = pixel_pos_x;
      p.y = pixel_pos_y;
      p.z = 0;
      tmp.polygon.points.push_back(p);
   }

   return tmp;
}
//transform a point in depthmap to the robot space
geometry_msgs::Pose Border::transformPtToDepthMap(double x, double y)
{
   int pixel_pos_x;
   int pixel_pos_y;

   x = x * 1000;
   y = y * 1000;
   pixel_pos_x = (int) (ax * x + bx);
   pixel_pos_y = (int) (ay * y + by);
   geometry_msgs::Pose p;
   p.position.x = pixel_pos_x;
   p.position.y = pixel_pos_y;

   return p;
}
//transform a point in depthmap to the robot space
geometry_msgs::Pose Border::transformPtToRobotSpace(int px, int py)
{
   readParamsDepthMap();
   double x = (px - bx) / ax;
   x = x / 1000.0;
   double y = (py - by) / ay;
   y = y / 1000.0;
   geometry_msgs::Pose p;
   p.position.x = x;
   p.position.y = y;
   p.position.z = 0.0;

   return p;
}