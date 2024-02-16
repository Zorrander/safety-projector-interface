#include "border/DynamicBorder.hpp"


DynamicBorder::DynamicBorder(ros::NodeHandle *nh_, std::string r_id, std::string z, std::string b_topic, std_msgs::ColorRGBA b_color, bool filling, int thic, ros::Duration life, bool track):
nh(*nh_),
it_(nh)
{
   request_id = r_id;
   zone = z;
   border_color = b_color;
   is_filled = filling;
   thickness = thic;
   lifetime = life;
   track_violations = track;
   base_link.subscribe(nh, "/coords/base_link", 1);
   link_4.subscribe(nh, "/coords/link_4", 1);
   link_5.subscribe(nh, "/coords/link_5", 1);
   tool_0.subscribe(nh, "/coords/tool0", 1);
   flange.subscribe(nh, "/coords/flange", 1);
   sync.reset(new Sync(MySyncPolicy(10), base_link, link_4, link_5, tool_0, flange));      
   sync->registerCallback(boost::bind(&DynamicBorder::callbackJoints3D, this, _1, _2, _3, _4, _5));
   dm_sub_ = it_.subscribe("/detection/depth_map", 1,&DynamicBorder::depthMapCallback, this);
   //pub_safety_colored = it_.advertise("/safety_line/line_proj", 1);
   pub_safety_inside = it_.advertise("/safety_line/inside_zone", 1);
   pub_border_poly = nh.advertise<geometry_msgs::PolygonStamped>("execution/safety/integration/topics/dynamic_safety_border",1);
   pub_pose_violation = nh.advertise<geometry_msgs::PoseStamped>("/projector_interface/pose_violation",1);
   pub_border_projection = nh.advertise<unity_msgs::BorderProj>("/projector_interface/display_dynamic_border",1);
   pub_border_violation = nh.advertise<integration::SafetyBorderViolation>("/execution/projector_interface/integration/topics/safety_border_violation_events",1);
   location_violation.position.x = 0.0;
   location_violation.position.y = 0.0;
   location_violation.position.z = 0.0;
   location_violation.orientation.x = 0.0;
   location_violation.orientation.y = 0.0;
   location_violation.orientation.z = 0.0;
   location_violation.orientation.w = 0.0;
   l_points.clear();
   prev_l_points.clear();
   readParamsDepthMap();
   cv_image = cv::Mat::zeros(1024,1024,CV_8UC1);
   safety_line_mask = cv::Mat::zeros(1024,1024,CV_8UC1);
   sf_line_inside = cv::Mat::zeros(1024,1024,CV_8UC1);
   sf_line_colored = cv::Mat::zeros(1024,1024,CV_8UC3);
   baseline_dm = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   sf_line = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   first_baseline = false;
   got_mask = false;
   threshold_depth_inf = 0.1;
   joints_moving = false;
   threshold_joints = 0.0001;
   lock_zone = false;
}

DynamicBorder::DynamicBorder(const DynamicBorder &cpctr):
nh(cpctr.nh),
it_(nh)
{
   this->request_id = cpctr.request_id;
   this->zone = cpctr.zone;
   this->border_color = cpctr.border_color;
   this->is_filled = cpctr.is_filled;
   this->thickness = cpctr.thickness;
   this->lifetime = cpctr.lifetime;
   this->track_violations = cpctr.track_violations;

}

void DynamicBorder::callbackJoints3D(geometry_msgs::TransformStampedConstPtr ts_bl, geometry_msgs::TransformStampedConstPtr ts_l4, 
                                    geometry_msgs::TransformStampedConstPtr ts_l5, geometry_msgs::TransformStampedConstPtr ts_tool, 
                                    geometry_msgs::TransformStampedConstPtr ts_flange)
{
   cv_image = cv::Mat::zeros(1024,1024,CV_8UC1);
   // ros::WallTime start_, end_;
   // start_ = ros::WallTime::now();
   cv::Mat res_contours;
   l_points.clear();
   geometry_msgs::Point bl;
   geometry_msgs::Point l4;
   geometry_msgs::Point l5;
   geometry_msgs::Point tool;
   geometry_msgs::Point flg;
   float diff_x = 0.0;
   float diff_y = 0.0;
   float diff_z = 0.0;
   bl.x = ts_bl->transform.translation.x;
   bl.y = ts_bl->transform.translation.y;
   bl.z = ts_bl->transform.translation.z;
   l4.x = ts_l4->transform.translation.x;
   l4.y = ts_l4->transform.translation.y;
   l4.z = ts_l4->transform.translation.z;
   l5.x = ts_l5->transform.translation.x;
   l5.y = ts_l5->transform.translation.y;
   l5.z = ts_l5->transform.translation.z;
   tool.x = ts_tool->transform.translation.x;
   tool.y = ts_tool->transform.translation.y;
   tool.z = ts_tool->transform.translation.z;
   flg.x = ts_flange->transform.translation.x;
   flg.y = ts_flange->transform.translation.y;
   flg.z = ts_flange->transform.translation.z;
   l_points.push_back(bl);
   l_points.push_back(l4);
   l_points.push_back(l5);
   l_points.push_back(tool);
   l_points.push_back(flg);
   //std::cout<<"diff x : "<<diff_x<<"\n";
   //std::cout<<"diff y : "<<diff_y<<"\n";
   //std::cout<<"diff z : "<<diff_z<<"\n";
   bool stop = false;
   for(int i = 0; i < l_points.size(); i++)
   {
      if(prev_l_points.size() > 0)
      {
         diff_x = std::abs(l_points[i].x - prev_l_points[i].x);
         diff_y = std::abs(l_points[i].y - prev_l_points[i].y);
         diff_z = std::abs(l_points[i].z - prev_l_points[i].z);
         //std::cout<<"diff x : "<<diff_x<<"\n";
         //std::cout<<"diff y : "<<diff_y<<"\n";
         //std::cout<<"diff z : "<<diff_z<<"\n";
         if((diff_x > threshold_joints || diff_y > threshold_joints || diff_z > threshold_joints) && !stop)
         {
            joints_moving = true;
            stop = true;
         }
      }
   }
   if(!stop)
   {
      joints_moving = false;
   }
   //std::cout<<"moving : "<<joints_moving<<"\n";

   drawBorder(ts_bl->header);
   prev_l_points.clear();
   for(geometry_msgs::Point p : l_points)
   {
      prev_l_points.push_back(p);
   }
   // end_ = ros::WallTime::now();
   // double execution_time = (end_ - start_).toNSec() * 1e-6;
   // ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
}

void DynamicBorder::depthMapCallback(const sensor_msgs::ImageConstPtr& msg_dm)
{
   ros::WallTime start_, end_;
   start_ = ros::WallTime::now();
   sf_line = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   if(track_violations)
   {
      cv::Mat depth_map = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
      cv_bridge::CvImagePtr cv_ptr;
      cv::Mat res_bl = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
      cv::Mat res_dm = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
      cv::Mat tmp = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
      bool is_cluster = false;
      try
      {
         cv_ptr = cv_bridge::toCvCopy(msg_dm, sensor_msgs::image_encodings::TYPE_32FC1);
         depth_map = cv_ptr->image.clone();
         if(!first_baseline)
         {
            baseline_dm = cv_ptr->image.clone();
            first_baseline = true;
         }
         if(first_baseline && joints_moving && lock_zone)
         {
            //std::cout<<"update !\n";
            depth_map.copyTo(baseline_dm,sf_line_inside);
         }
         if(got_mask && first_baseline)
         {
            baseline_dm.copyTo(res_bl,safety_line_mask);
            depth_map.copyTo(res_dm,safety_line_mask);
            cv::subtract(res_dm,res_bl,sf_line);
            cv::Mat tmp2;
            cv::Mat tmp3;
            //transform image to apply filter
            cv::cvtColor(sf_line,tmp2,cv::COLOR_GRAY2RGB);
            tmp2.convertTo(tmp2, CV_8U, 255.0);
            cv::medianBlur(tmp2,tmp3,(3,3));
            cv::Mat tmp4;
            //back to depth
            cv::cvtColor(tmp3,tmp4,cv::COLOR_RGB2GRAY);
            tmp4.convertTo(tmp, CV_32FC1, 1/255.0);
            is_cluster = detectCluster(tmp);
            if(is_cluster)
            {
               //std::cout<<"cluster !\n";
               test_pose.header = msg_dm->header;
               test_pose.header.frame_id = "base_robot_master";
               test_pose.pose = location_violation;
               pub_pose_violation.publish(test_pose);
               integration::SafetyBorderViolation msg_border;
               msg_border.header = msg_dm->header;
               msg_border.header.frame_id = "base_robot_master";
               msg_border.request_id = request_id;
               border.header = msg_dm->header;
               border.header.frame_id = "base_robot_master";
               msg_border.initial_border = border;
               msg_border.target_location = location_violation;
               pub_border_violation.publish(msg_border);
            }
         }
      }
      catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }
      cv::imshow(OPENCV_WINDOW, tmp);
      cv::waitKey(1);
   }
   
   end_ = ros::WallTime::now();
   double execution_time = (end_ - start_).toNSec() * 1e-6;
   //ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
}

void DynamicBorder::echo()
{
   std::cout<<"hello\n";
}

void DynamicBorder::drawBorder(std_msgs::Header header)
{
   int pixel_pos_x;
   int pixel_pos_y;
   safety_line_mask = cv::Mat::zeros(1024,1024,CV_8UC1);
   got_mask = false;
   sf_line_inside = cv::Mat::zeros(1024,1024,CV_8UC1);
   lock_zone = false;
   sf_line_colored = cv::Mat::zeros(1024,1024,CV_8UC3);
   geometry_msgs::PolygonStamped border_rgb_frame;
   border_rgb_frame.polygon.points.clear();
   border_rgb_frame.header = header;
   border_rgb_frame.header.frame_id = "base_robot_master";
   //transform to depth map space to be able to find and draw the hull
   for(int i = 0; i < l_points.size(); i++)
   {
      geometry_msgs::Pose pt = transformPtToDepthMap(l_points[i].x,l_points[i].y);
      cv::Point center(pt.position.x, pt.position.y);
      circle(cv_image, center,100, cv::Scalar(255, 255, 0), 1);
   }
   std::vector<std::vector<cv::Point>> contours;
   std::vector<cv::Vec4i> hierarchy;
   cv::findContours(cv_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
   std::vector<std::vector<cv::Point>> hull( contours.size() );
   for( size_t i = 0; i < contours.size(); i++ )
   {
      convexHull( contours[i], hull[i] );
   }
   drawContours(safety_line_mask, hull, 0, cv::Scalar(255, 255, 255),5);
   got_mask = true;
   if(is_filled)
   {
      drawContours(sf_line_colored, hull, 0, cv::Scalar(border_color.b*255, border_color.g*255, border_color.r*255),cv::FILLED);
   }
   else
   {
      drawContours(sf_line_colored, hull, 0, cv::Scalar(border_color.b*255, border_color.g*255, border_color.r*255),thickness);
   }
   
   drawContours(sf_line_inside, hull, 0, cv::Scalar(255, 255, 255),cv::FILLED);
   lock_zone = true;
   //create polygon from hull
   for(int i = 0; i < hull[0].size(); i++)
   {
      geometry_msgs::Point32 p;
      p.x = hull[0][i].x;
      p.y = hull[0][i].y;
      border_rgb_frame.polygon.points.push_back(p);
   }
   //Close the hull by adding first point to last position
   geometry_msgs::Point32 p;
   p.x = hull[0][0].x;
   p.y = hull[0][0].y;
   border_rgb_frame.polygon.points.push_back(p);
   
   transformToRobotSpace(border_rgb_frame);
   publishHulls();
   
}

void DynamicBorder::publishHulls()
{
   unity_msgs::BorderProj border_msg;
   border_msg.zone = zone;
   sensor_msgs::ImagePtr msg_col;
   msg_col = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC3, sf_line_colored).toImageMsg();
   border_msg.img = *msg_col;
   pub_border_projection.publish(border_msg);
   sensor_msgs::ImagePtr msg_inside;
   msg_inside = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC1, sf_line_inside).toImageMsg();
   pub_safety_inside.publish(msg_inside);
}

void DynamicBorder::transformToRobotSpace(geometry_msgs::PolygonStamped cv_poly)
{
   border.polygon.points.clear();
   for(int i = 0; i < cv_poly.polygon.points.size(); i++)
   {
      int px = cv_poly.polygon.points[i].x;
      int py = cv_poly.polygon.points[i].y;
      double x = (px - bx) / ax;
      x = x / 1000.0;
      double y = (py - by) / ay;
      y = y / 1000.0;
      geometry_msgs::Point32 p;
      p.x = x;
      p.y = y;
      p.z = 0.0;
      border.polygon.points.push_back(p);
   }
   border.header = cv_poly.header;
   pub_border_poly.publish(border);
}

bool DynamicBorder::detectCluster(cv::Mat img)
{
   bool res = false;
   int best_sum = 0;
   int sum = 0;
   int i = 0;
   int j = 0;
   float detect;
   cv::Point p_v;
   for(int i = 0; i < img.rows; i++)
   {
      for(int j = 0; j < img.cols; j++)
      {
         float t = img.at<float>(i,j);
         int k = 0;
         int l = 0;
         //cout<<t<<"\n";
         if(img.at<float>(i,j) > threshold_depth_inf)
         {
            sum = 0;
            for(int k = 0; k < 7 ; k++)
            {
              for(int l = 0; l < 7; l++)
              {
                  if(img.at<float>(i+k,j+l) > threshold_depth_inf)
                  {
                     //detect = img.at<float>(i+k,j+l);
                     //cout<<detect<<"\n";
                     //cout<<"k "<<k<<"\n";
                     //cout<<"l "<<l<<"\n";
                     sum++;
                  }
              }
            }
         }
         if(sum >= 45)
         {
            p_v.x = j + 2;
            p_v.y = i + 2;
            test_pose.pose = transformPtToRobotSpace(j+2,i+2);
            location_violation = transformPtToRobotSpace(j+2,i+2);
            res = true;
         }
         if(sum > best_sum)
         {
            best_sum = sum;
         }
      }
   }
   //std::cout<<"sum : "<<best_sum<<"\n";
   return res;
}
