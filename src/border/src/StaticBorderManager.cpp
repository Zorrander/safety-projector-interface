#include "border/StaticBorderManager.hpp"


StaticBorderManager::StaticBorderManager(ros::NodeHandle *nh_, StaticBorder sb):
nh(*nh_),
it_(nh)
{
   //ros::param::get("highest_depth", highest_depth);
   dm_sub_ = it_.subscribe("/detection/depth_map", 1,&StaticBorderManager::depthMapCallback, this);  
   pub_border_projection = nh.advertise<unity_msgs::LBorder>("/projector_interface/display_static_border",1);
   pub_border_violation = nh.advertise<integration::SafetyBorderViolation>("/execution/projector_interface/integration/topics/safety_border_violation",1);
   pub_pose_violation = nh.advertise<geometry_msgs::PoseStamped>("/projector_interface/pose_violation",1);
   pub_borders_vacancy = nh.advertise<unity_msgs::LStaticBorder>("/projector_interface/borders_vacancy",1);
   list_proj.clear();
   list_content.clear();
   borders.clear();
   borders.push_back(sb);
   redraw = true;
   first_init = true;
   safety_line_mask = cv::Mat::zeros(1024,1024,CV_8U);
   sf_line_colored = cv::Mat::zeros(1024,1024,CV_8UC3);
   baseline_dm = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   sf_line = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   depth_map = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   res_dm = cv::Mat(1024, 1024, CV_32F,cv::Scalar(std::numeric_limits<float>::min()));
   highest_depth = 0.577727;
   threshold_depth_inf = 0.1;
   first_baseline = true;
   crossed = false;
   index = -1;
   previous_index = -1;
   change_color = false;
   minmax_values.clear();
}


void StaticBorderManager::addBorder(StaticBorder sb)
{
   borders.push_back(sb);
   redraw = true;
}

void StaticBorderManager::drawBorder(string id_draw)
{
   cv::Mat sf_line_col = cv::Mat::zeros(1024,1024,CV_8UC3);
   cv::Mat sf_line_mask = cv::Mat::zeros(1024,1024,CV_8U);
   int test = id_draw.compare("-1");
   if(test != 0)
   {
      list_proj.clear();
      //create border depending on the projection zone
      for(StaticBorder i : borders)
      {
         if(list_proj.size() == 0)
         {
            Projection pj;
            pj.id = i.getId();
            pj.img = sf_line_col;
            pj.mask = sf_line_mask;
            pj.zone = i.getZone();
            list_proj.push_back(pj);
         }
         else
         {
            for(Projection j : list_proj)
            {
               int cmp = i.getZone().compare(j.zone);
               if(cmp != 0)
               {
                  Projection pj;
                  pj.img = sf_line_col;
                  pj.mask = sf_line_mask;
                  pj.zone = i.getZone();
                  list_proj.push_back(pj);
               }
            }
         }
      }
      //dispatch borders by zones
      for(Projection i : list_proj)
      {
         for(StaticBorder j : borders)
         {
            int cmp = i.zone.compare(j.getZone());
            if(cmp == 0)
            {
               i.img = i.img + j.drawBorder().clone();
               i.mask = i.mask + j.drawMask().clone();
            }
         }
      }
   }
   else
   {
      for(Projection i : list_proj)
      {
         for(StaticBorder j : borders)
         {
            int cmp = i.zone.compare(j.getZone());
            if(cmp == 0)
            {
               i.img = i.img + j.drawBorder().clone();
               //i.mask = i.mask + j.drawMask().clone();
            }
         }
      }
   }
}

void StaticBorderManager::detectContent(cv::Mat current_img)
{
   cv::Mat big = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   if(first_init)
   {
      list_content.clear();
      minmax_values.clear();
      for(int i = 0; i < borders.size(); i++)
      {
         BorderContent bc;
         bc.id = borders[i].getId();
         bc.center = borders[i].getCenter();
         int x = static_cast<int>(bc.center.x);
         int y = static_cast<int>(bc.center.y);
         bc.baseline_depth = highest_depth;
         std::cout<<"baseline depth : "<<bc.baseline_depth<<"\n";
         /*for(int k = bc.center.y -5; k < bc.center.y + 5;k++)
         {
            for(int l = bc.center.x -5; k < bc.center.x + 5;l++)
            {
               float tmp = current_img.at<float>(l,k);
               std::cout<<"depth : "<<tmp<<"\n";
            }
         }*/
         list_content.push_back(bc);
      }
      minmax_values = getMinMax();
      first_init = false;
   }
   
   for(BorderContent i : list_content)
   {
      cv::Mat mask_center = cv::Mat::zeros(1024,1024,CV_8UC1);
      //cv::Mat detect_bl = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
      cv::Mat detect_current = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
      cv::Point tmp;
      tmp.x = i.center.x;
      tmp.y = i.center.y;
      circle(mask_center, tmp,7, cv::Scalar(255, 255, 255), -1);
      //circle(current_img, tmp,7, cv::Scalar(255, 255, 255), -1);
      //cv::Mat mask_tmp = cv::Mat::zeros(1024,1024,CV_8U);
      //cv::Mat tmp_m;

      current_img.copyTo(detect_current,mask_center);
      cv::Mat result = enhanceDepth(detect_current,i.baseline_depth);
      big = big + result.clone();
      bool occ = detectClusterOpt(result,4,3,minmax_values[0],minmax_values[1]);
      i.occupied = occ;
   }
   cv::imshow(OPENCV_TEST, big);
   cv::waitKey(2);
}

void StaticBorderManager::publishOccupancy()
{
   unity_msgs::LStaticBorder msg_all;
   for(BorderContent i : list_content)
   {
      unity_msgs::BorderID msg;
      msg.center.x = i.center.x;
      msg.center.y = i.center.y;
      msg.id = i.id;
      msg.occupy = i.occupied;
      msg_all.l_borders.push_back(msg);
   }
   pub_borders_vacancy.publish(msg_all);
}

void StaticBorderManager::depthMapCallback(const sensor_msgs::ImageConstPtr& msg_dm)
{
   //std::cout<<"callback\n";
   ros::Time begin = ros::Time::now();
   sf_line = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   //geometry_msgs::PolygonStamped disp = borders[0].getBorderCameraSpace();
   //std::cout<<borders.size()<<"\n";
   if(redraw)
   {
      drawBorder(index_redraw);
      safety_line_mask = cv::Mat::zeros(1024,1024,CV_8U);
      sf_line_colored = cv::Mat::zeros(1024,1024,CV_8UC3);
      for(Projection i : list_proj)
      {
         sf_line_colored = sf_line_colored + i.img.clone();
         safety_line_mask = safety_line_mask + i.mask.clone();
      }
      //cv::Mat res;
      //cv::cvtColor(sf_line_colored,res,cv::COLOR_RGB2GRAY);
      //res.convertTo(safety_line_mask, CV_8U, 255.0);
      publishBorder();
      redraw = false;
      first_baseline = true;
      first_init = true;
   }
   
   cv_bridge::CvImagePtr cv_ptr;
   cv::Mat res_bl = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   res_dm = cv::Mat(1024, 1024, CV_32F,cv::Scalar(std::numeric_limits<float>::min()));
   cv::Mat cluster_img = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   cv::Mat enhanced = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   bool is_cluster = false;
   
   try
   {
      cv_ptr = cv_bridge::toCvCopy(msg_dm, sensor_msgs::image_encodings::TYPE_32FC1);
      depth_map = cv_ptr->image.clone();
      //detectContent(depth_map);
      //publishOccupancy();
      /*for(BorderContent i : list_content)
      {
         cv::Point tmp;
         tmp.x = i.center.x;
         tmp.y = i.center.y;
         circle(depth_map, tmp,5, cv::Scalar(255, 255, 255), -1);
      }*/
      /*for(int i = 0;i < disp.polygon.points.size();i++)
       {
          cv::Point tmp;
          tmp.x = disp.polygon.points[i].x;
          tmp.y = disp.polygon.points[i].y;
          circle(depth_map, tmp,5, cv::Scalar(255, 255, 255), -1);
          geometry_msgs::Point c = borders[0].getCenter();
          tmp.x = static_cast<int>(c.x);
          tmp.y = static_cast<int>(c.y);
          circle(depth_map, tmp,5, cv::Scalar(255, 255, 255), -1);
          //std::cout<<"x : "<<tmp.x<<"\n";
          //std::cout<<"y : "<<tmp.y<<"\n";

       }*/
      if(first_baseline)
      {
         baseline_dm = cv_ptr->image.clone();
         baseline_dm.copyTo(res_bl,safety_line_mask);
         first_baseline = false;
      }
      if(!first_baseline)
      {
         //detectContent(depth_map);
         //publishOccupancy();
         depth_map.copyTo(res_dm,safety_line_mask);
         enhanced = enhanceDepth(res_dm,highest_depth);
         cv::Mat tmp2;
         cv::Mat tmp3;
         cv::Mat tmp4;
         cv::cvtColor(enhanced,tmp2,cv::COLOR_GRAY2RGB);
         tmp2.convertTo(tmp2, CV_8U, 255.0);
         cv::medianBlur(tmp2,tmp3,(3,3));
         cv::cvtColor(tmp3,tmp4,cv::COLOR_RGB2GRAY);
         tmp4.convertTo(cluster_img, CV_32FC1, 1/255.0);
         is_cluster = detectCluster(enhanced,3,1);
         if(is_cluster)
         {
            //std::cout<<"cluster !\n";
            //fill border location violation just for display
            //pose_location.header = msg_dm->header;
            //pose_location.header.frame_id = "base";
            //pub_pose_violation.publish(pose_location);
            //fill msg that raise a border violation
            index = getViolatedBorder();
            
            if(borders[index].getTracking())
            {
               std::cout<<"cluster on border "<<index<<"\n";
               index_redraw = borders[index].getId();
               std_msgs::ColorRGBA c;
               c.a = 0.0;
               c.r = 1.0;
               c.g = 0.0;
               c.b = 0.0;
               borders[index].changeBorderColor(c);
               if(index != previous_index)
               {
                  c.a = 0.0;
                  c.r = 0.0;
                  c.g = 1.0;
                  c.b = 0.0;
                  borders[previous_index].changeBorderColor(c);
               }
               integration::SafetyBorderViolation msg_border;
               msg_border.header = msg_dm->header;
               msg_border.header.frame_id = "base";
               msg_border.request_id = borders[index].getId();
               //get violated border
               geometry_msgs::PolygonStamped bord = borders[index].getBorderRobotSpace();
               bord.header = msg_dm->header;
               bord.header.frame_id = "base";
               msg_border.initial_border = bord;
               msg_border.target_location = pose_location.pose;
               pub_border_violation.publish(msg_border);
               redraw = true;
               crossed = true;
               previous_index = index;
               prev_index_redraw = index_redraw;
               change_color = true;
            }
         }
         else
         {
            if(crossed)
            {
               std_msgs::ColorRGBA c;
               c.a = 0.0;
               c.r = 0.0;
               c.g = 1.0;
               c.b = 0.0;
               borders[index].changeBorderColor(c);
               redraw = true;
               crossed = false;
               //std::cout<<"redraw geen\n";
            }
         }
      }
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }
   ros::Time tmp_t = ros::Time::now();
   double t = tmp_t.toSec() - begin.toSec();
   //std::cout<<t<<"\n";
   //std::cout<<"inside\n";
   cv::imshow(OPENCV_TEST, enhanced);
   cv::waitKey(1);
}

float StaticBorderManager::getHighestPixel(cv::Mat img)
{
   float best = 0;
   for(int i = 0; i < img.rows; i++)
   {
      for(int j = 0; j < img.cols; j++)
      {
         float tmp = img.at<float>(i,j);
         if(tmp > best)
         {
            best = tmp;
         }
      }
   }
   return best;
}

cv::Mat StaticBorderManager::enhanceDepth(cv::Mat img, float thr)
{
   for(int i = 0; i < img.rows; i++)
   {
      for(int j = 0; j < img.cols; j++)
      {
         float tmp = img.at<float>(i,j);
         if(tmp > thr)
         {
            img.at<float>(i,j) = tmp * 1.3;
         }
         else
         {
            img.at<float>(i,j) = 0.0;
         }
      }
   }

   return img;
}

int StaticBorderManager::getViolatedBorder()
{
   float x_b = 3000;
   float y_b = 3000;
   float tot = 500;
   int best;
   for(int i = 0; i < borders.size(); i++)
   {
      //std::cout<<"i : "<<i<<"\n";
      //adding 2 missing corners
      geometry_msgs::PolygonStamped p = borders[i].getBorderCameraSpace();
      geometry_msgs::Point32 tr;
      tr.x = p.polygon.points[1].x;
      tr.y = p.polygon.points[0].y;
      geometry_msgs::Point32 bl;
      bl.x = p.polygon.points[0].x;
      bl.y = p.polygon.points[1].y;
      p.polygon.points.push_back(tr);
      p.polygon.points.push_back(bl);
      for(int j = 0; j < p.polygon.points.size(); j++)
      {
         float tmp_x = std::abs(p.polygon.points[j].x - loc.x);
         float tmp_y = std::abs(p.polygon.points[j].y - loc.y);
         float tmp_tot = tmp_x + tmp_y;  
         if(tmp_tot < tot)
         {
            tot = tmp_tot;
            best = i;
         }
      }
   }

   return best;
}

void StaticBorderManager::publishBorder()
{
   unity_msgs::LBorder msg;
   for(Projection i : list_proj)
   {
      unity_msgs::BorderProj border_msg;
      border_msg.zone = i.zone;
      sensor_msgs::ImagePtr msg_col;
      msg_col = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_8UC3, i.img).toImageMsg();
      border_msg.img = *msg_col;
      msg.list_borders.push_back(border_msg);
   }
   pub_border_projection.publish(msg);
}

std::vector<float> StaticBorderManager::getMinMax()
{
   int ref_min_y = 2000;
   int ref_max_y = 0;
   for(BorderContent i : list_content)
   {
      if(i.center.y < ref_min_y)
      {
         ref_min_y = i.center.y;
      }
      if(i.center.y > ref_max_y)
      {
         ref_max_y = i.center.y;
      }
   }
   std::vector<float> minmax;
   minmax.clear();
   minmax.push_back(ref_min_y);
   minmax.push_back(ref_max_y);

   return minmax;
}

bool StaticBorderManager::detectCluster(cv::Mat img, int kernel, int thr)
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
            for(int k = 0; k < kernel ; k++)
            {
              for(int l = 0; l < kernel; l++)
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
         if(sum >= ((kernel*kernel) - thr))
         {
            loc.x = j + 2;
            loc.y = i + 2;
            pose_location.pose = borders[0].transformPtToRobotSpace(loc.x,loc.y);
            //std::cout<<"violation x : "<<pose_location.pose.position.x<<"\n";
            //std::cout<<"violation y : "<<pose_location.pose.position.y<<"\n";
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

bool StaticBorderManager::detectClusterOpt(cv::Mat img, int kernel, int thr, int miny, int maxy)
{
   bool res = false;
   int best_sum = 0;
   int sum = 0;
   int i = miny-10;
   int j = 0;
   float detect;
   cv::Point p_v;
   while(i < maxy+15 && i > miny-15)
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
            for(int k = 0; k < kernel ; k++)
            {
              for(int l = 0; l < kernel; l++)
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
         if(sum >= ((kernel*kernel) - thr))
         {
            res = true;
         }
         if(sum > best_sum)
         {
            best_sum = sum;
         }
      }
      i++;
   }
   //std::cout<<"sum : "<<best_sum<<"\n";
   return res;
}