#include "border/StaticBorder.hpp"


StaticBorder::StaticBorder(std::string r_id, std::string z, geometry_msgs::PolygonStamped bord,std::string b_topic, std_msgs::ColorRGBA b_color, bool filling, int thic, ros::Duration life, bool track)
{
   readParamsDepthMap();
   request_id = r_id;
   zone = z;
   border_robot_space.header = bord.header;
   border_robot_space.polygon.points.clear();
   border_camera_space.header = bord.header;
   border_camera_space.polygon.points.clear();
   for(int i  = 0; i < bord.polygon.points.size(); i++)
   {
      border_robot_space.polygon.points.push_back(bord.polygon.points[i]);
      
   }
   border_camera_space = transformToDepthMap(border_robot_space);
   border_color = b_color;
   is_filled = filling;
   thickness = thic;
   lifetime = life;
   track_violations = track;
   border_mask = cv::Mat::zeros(1024,1024,CV_8U);
}

StaticBorder::StaticBorder(const StaticBorder &cpctr)
{
   this->request_id = cpctr.request_id;
   this->zone = cpctr.zone;
   this->border_robot_space = cpctr.border_robot_space;
   this->border_camera_space = cpctr.border_camera_space;
   this->border_color = cpctr.border_color;
   this->is_filled = cpctr.is_filled;
   this->thickness = cpctr.thickness;
   this->lifetime = cpctr.lifetime;
   this->track_violations = cpctr.track_violations;

}

geometry_msgs::PolygonStamped StaticBorder::getBorderCameraSpace()
{
   return border_camera_space;
}

geometry_msgs::PolygonStamped StaticBorder::getBorderRobotSpace()
{
   return border_robot_space;
}

cv::Mat StaticBorder::drawBorder()
{
   sf_line_colored = cv::Mat::zeros(1024,1024,CV_8UC3);
   geometry_msgs::Point32 tl;
   tl.x = 5000;
   tl.y = 5000;
   geometry_msgs::Point32 br;
   br.x = 0;
   br.y = 0;
   for(geometry_msgs::Point32 i : border_camera_space.polygon.points)
   {
      //std::cout<<" x : "<<i.x<<" y : "<<i.y<<"\n";
      //std::cout<<"br x : "<<br.x<<" br y : "<<br.y<<"\n";
      if(i.x < tl.x && i.y < tl.y)
      {
         tl.x = i.x;
         tl.y = i.y;
      }
      if(i.x > br.x && i.y > br.y)
      {
         br.x = i.x;
         br.y = i.y;
      }
      //std::cout<<" x : "<<i.x<<" y : "<<i.y<<"\n";
   }
   cv::Point top_l(static_cast<int>(tl.x),static_cast<int>(tl.y));
   cv::Point bottom_r(static_cast<int>(br.x),static_cast<int>(br.y));
   cv::rectangle(sf_line_colored,top_l,bottom_r,cv::Scalar(border_color.b*255, border_color.g*255, border_color.r*255),thickness,cv::LINE_8);
   //std::cout<<"color : RGB "<<border_color.r<<border_color.g<<border_color.b<<"\n";
   //std::cout<<"tl x : "<<tl.x<<" tl y : "<<tl.y<<"\n";
   //std::cout<<"br x : "<<br.x<<" br y : "<<br.y<<"\n";


   return sf_line_colored;

}

cv::Mat StaticBorder::drawMask()
{
   safety_line_mask = cv::Mat::zeros(1024,1024,CV_8UC1);
   geometry_msgs::Point32 tl;
   tl.x = 5000;
   tl.y = 5000;
   geometry_msgs::Point32 br;
   br.x = 0;
   br.y = 0;
   for(geometry_msgs::Point32 i : border_camera_space.polygon.points)
   {
      if(i.x < tl.x && i.y < tl.y)
      {
         tl.x = i.x;
         tl.y = i.y;
      }
      if(i.x > br.x && i.y > br.y)
      {
         br.x = i.x;
         br.y = i.y;
      }
   }
   cv::Point top_l(static_cast<int>(tl.x),static_cast<int>(tl.y));
   cv::Point bottom_r(static_cast<int>(br.x),static_cast<int>(br.y));
   cv::rectangle(safety_line_mask,top_l,bottom_r,cv::Scalar(255,255,255),3,cv::LINE_8);


   return safety_line_mask;
}

void StaticBorder::changeBorderColor(std_msgs::ColorRGBA col)
{
   border_color = col;
}

geometry_msgs::Point StaticBorder::getCenter()
{
   geometry_msgs::Point p;
   p.x = static_cast<double>((border_camera_space.polygon.points[0].x + border_camera_space.polygon.points[1].x)/2);
   p.y = static_cast<int>((border_camera_space.polygon.points[0].y + border_camera_space.polygon.points[1].y)/2);

   return p;
}

std::string StaticBorder::getZone()
{
   return zone;
}

std::string StaticBorder::getId()
{
   return request_id;
}

bool StaticBorder::getTracking()
{
   return track_violations;
}