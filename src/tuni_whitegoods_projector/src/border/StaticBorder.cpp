/*
Class to create StaticBorder
*/
#include "border/StaticBorder.h"

//The parameters are defined in OpenFlow
//id : id of the border
//z : zone where to display the border
// pos_row, pos_col : the virtual position of this border in a layout
// bord : the border (top-left, bottom-right) of the border
//b_color : color of the border
// filling : if the border should be filled
// thic : thickness of the border
//life : lifetime of the border. Not implemented here because not really useful
//track : if we track the border (monitor any crossing)
StaticBorder::StaticBorder(std::string r_id, std::string z, int pos_row, int pos_col, geometry_msgs::PolygonStamped bord,std::string b_topic, std_msgs::ColorRGBA b_color, bool filling, int thic, ros::Duration life, bool track)
{
   readParamsDepthMap();
   request_id = r_id;
   zone = z;
   position_row = pos_row;
   position_col = pos_col;
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
   this->position_col = cpctr.position_col;
   this->position_row = cpctr.position_row;
   this->border_robot_space = cpctr.border_robot_space;
   this->border_camera_space = cpctr.border_camera_space;
   this->border_color = cpctr.border_color;
   this->is_filled = cpctr.is_filled;
   this->thickness = cpctr.thickness;
   this->lifetime = cpctr.lifetime;
   this->track_violations = cpctr.track_violations;

}
//get the diagonal of the border. Used for hand detection if a hand's location is less than diagonal*factor then it throws a violation
float StaticBorder::getBorderDiagonal()
{
   float x_term;
   float y_term;
   x_term = static_cast<float>(border_camera_space.polygon.points[1].x - border_camera_space.polygon.points[0].x);
   y_term = static_cast<float>(border_camera_space.polygon.points[1].y - border_camera_space.polygon.points[0].y);
   float dist = sqrt(pow(x_term,2) + pow(y_term,2));

   return dist;
}
//get the border in the camera space
geometry_msgs::PolygonStamped StaticBorder::getBorderCameraSpace()
{
   return border_camera_space;
}
//get the border in robot space
geometry_msgs::PolygonStamped StaticBorder::getBorderRobotSpace()
{
   geometry_msgs::Point32 tl;
   tl.x = border_robot_space.polygon.points[0].x;
   tl.y = border_robot_space.polygon.points[0].y;
   geometry_msgs::Point32 br;
   br.x = border_robot_space.polygon.points[1].x;
   br.y = border_robot_space.polygon.points[1].y;
   geometry_msgs::Point32 tr;
   tr.x = border_robot_space.polygon.points[0].x;
   tr.y = border_robot_space.polygon.points[1].y;
   geometry_msgs::Point32 bl;
   bl.x = border_robot_space.polygon.points[1].x;
   bl.y = border_robot_space.polygon.points[0].y;
   border_robot_space.polygon.points.clear();
   border_robot_space.polygon.points.push_back(tl);
   border_robot_space.polygon.points.push_back(tr);
   border_robot_space.polygon.points.push_back(br);
   border_robot_space.polygon.points.push_back(bl);
   
   return border_robot_space;
}

//draw a border
cv::Mat StaticBorder::drawBorder() 
{
   sf_line_colored = cv::Mat::zeros(1024,1024,CV_8UC3);
   cv::Point top_l(static_cast<int>(border_camera_space.polygon.points[0].x),
                   static_cast<int>(border_camera_space.polygon.points[0].y));
   cv::Point bottom_r(static_cast<int>(border_camera_space.polygon.points[1].x),
                      static_cast<int>(border_camera_space.polygon.points[1].y));
   ROS_INFO("Border x,y coordinates: (%i, %i), (%i, %i)", top_l.x, top_l.y, bottom_r.x, bottom_r.y);
   cv::rectangle(sf_line_colored,top_l,bottom_r,cv::Scalar(border_color.b*255, border_color.g*255, border_color.r*255),thickness,cv::LINE_8);
   // cv::imshow("sb:drawBorder", sf_line_colored);
   // cv::waitKey(0);
   return sf_line_colored;
}

//draw the mask of the border. Used for detection
cv::Mat StaticBorder::drawMask() 
{
   safety_line_mask = cv::Mat::zeros(1024,1024,CV_8UC1);
   cv::Point top_l(static_cast<int>(border_camera_space.polygon.points[0].x),
                   static_cast<int>(border_camera_space.polygon.points[0].y));
   cv::Point bottom_r(static_cast<int>(border_camera_space.polygon.points[1].x),
                      static_cast<int>(border_camera_space.polygon.points[1].y));
   
   cv::rectangle(safety_line_mask,top_l,bottom_r,cv::Scalar(255,255,255),3,cv::LINE_8);

   return safety_line_mask;
}

//change the color of the border
void StaticBorder::changeBorderColor(std_msgs::ColorRGBA& col)
{
   border_color.r = col.r;
   border_color.g = col.g;
   border_color.b = col.b;
   border_color.a = col.a;
}

//change the thickness of the border
void StaticBorder::changeThickness(int thic)
{
   thickness = thic;
}

//get the center of the border
geometry_msgs::Point StaticBorder::getCenter()
{
   geometry_msgs::Point p;
   p.x = static_cast<double>((border_camera_space.polygon.points[0].x + border_camera_space.polygon.points[1].x)/2);
   p.y = static_cast<int>((border_camera_space.polygon.points[0].y + border_camera_space.polygon.points[1].y)/2);

   return p;
}

//return the zone where the border should be displayed
std::string StaticBorder::getZone() const
{
   return zone;
}
//return the id of the border
std::string StaticBorder::getId()
{
   return request_id;
}
//return if the border should be monitored
bool StaticBorder::getTracking()
{
   return track_violations;
}
//get the coloumn location of the border within the virtual layout
int StaticBorder::getCol()
{
   return position_col;
}
//get the row location of the border within the virtual layout
int StaticBorder::getRow()
{
   return position_row;
}
//get the border color
std_msgs::ColorRGBA StaticBorder::getColor()
{
   return border_color;
}


