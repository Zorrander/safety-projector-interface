/*
Class to create StaticBorder
*/
#include "staticBorder.h"

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
   request_id = r_id;
   zone = z;
   position_row = pos_row;
   position_col = pos_col;
   border_robot_space.header = bord.header;
   border_robot_space.polygon.points.clear();
   for(int i  = 0; i < bord.polygon.points.size(); i++)
   {
      border_robot_space.polygon.points.push_back(bord.polygon.points[i]);
      ROS_INFO("pt: (%f, %f)", bord.polygon.points[i].x, bord.polygon.points[i].y);
   }
   ROS_INFO("InitBorder robt x,y coordinates: (%f, %f), (%f, %f)", border_robot_space.polygon.points[0].x, border_robot_space.polygon.points[0].y, border_robot_space.polygon.points[1].x, border_robot_space.polygon.points[1].y);
   border_color = b_color;
   is_filled = filling;
   thickness = thic;
   lifetime = life; 
   track_violations = track;
   left_hand_crossed = false;
   right_hand_crossed = false;
   booked = false;
}


//draw a border
cv::Mat StaticBorder::drawBorder() 
{
   sf_line_colored = cv::Mat::zeros(1080, 1920,CV_8UC3);
   cv::rectangle(sf_line_colored, top_left_cam_point, bottom_right_cam_point, cv::Scalar(border_color.b*255, border_color.g*255, border_color.r*255), thickness,cv::LINE_8);
   return sf_line_colored;
}


//change the color of the border
void StaticBorder::changeBorderColor(std_msgs::ColorRGBA& col)
{
   border_color.r = col.r;
   border_color.g = col.g;
   border_color.b = col.b;
   border_color.a = col.a;
}


//change the thickness oZf the border
void StaticBorder::changeThickness(int thic)
{
   thickness = thic;
}

//get the coloumn location of the border within the virtual layout
void StaticBorder::book()
{
   booked = true;
}
//get the row location of the border within the virtual layout
void StaticBorder::release()
{
   booked = false;
}

//get the diagonal of the border. Used for hand detection if a hand's location is less than diagonal*factor then it throws a violation
float StaticBorder::getBorderDiagonal()
{
   float dist = sqrt(pow((bottom_right_cam_point.x - top_left_cam_point.x),2) + pow((bottom_right_cam_point.y - top_left_cam_point.y),2));
   return dist;
}

//get the center of the border
cv::Point StaticBorder::getCenter()
{
   cv::Point p;
   p.x = (top_left_cam_point.x + bottom_right_cam_point.x)/2;
   p.y = (top_left_cam_point.y + bottom_right_cam_point.y)/2;

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



