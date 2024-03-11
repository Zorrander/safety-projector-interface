/*
Class that manage static borders. It display and monitors them.
*/
#include "border/StaticBorderManager.h"


// Params are almost the same as in StaticBorder
//adjacent : if true then adjacent borders must be booked too
//sf_factor : safety_factor for the hand detection.
StaticBorderManager::StaticBorderManager(ros::NodeHandle *nh_, int rows, int cols, float sf_factor, bool adjacent, std_msgs::ColorRGBA status_booked, std_msgs::ColorRGBA status_free, std_msgs::ColorRGBA status_operator):
nh(*nh_),
it_(nh)
{
   std::cout<<"StaticBorderManager starting\n";
   //ros::param::get("highest_depth", highest_depth);
   service_borders = nh.advertiseService("/execution/projector_interface/integration/services/list_static_border_status", &StaticBorderManager::getBordersService, this);
   dm_sub_ = it_.subscribe("/detection/depth_map", 1,&StaticBorderManager::depthMapCallback, this);  
   sub_hand_detection = nh.subscribe("/hand_tracking/dm/coordinates", 1,&StaticBorderManager::handTrackingCallback, this);
   pub_border_projection = nh.advertise<unity_msgs::LBorder>("/projector_interface/display_static_border",1);
   pub_border_violation = nh.advertise<integration::SafetyBorderViolation>("/execution/projector_interface/integration/topics/safety_border_violation",1);
   pub_pose_violation = nh.advertise<geometry_msgs::PoseStamped>("/projector_interface/pose_violation",1);
   pub_borders_vacancy = nh.advertise<unity_msgs::LStaticBorder>("/projector_interface/borders_vacancy",1);
   pub_border_polygon = nh.advertise<geometry_msgs::PolygonStamped>("/border_violated",1);
   s_rows = rows;
   s_cols = cols;
   safety_factor = sf_factor;
   adj = adjacent;
   stat_booked = status_booked;
   stat_free = status_free;
   stat_operator = status_operator;
   ros::param::get("calibration_folder", calibration_folder);
   name_bl = calibration_folder + "baseline";
   //baseline_dm = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   //read the baseline image generated during calibration
   readRawImage(baseline_dm, name_bl);
   list_proj.clear();
   borders.clear();
   borders_status.clear();
   borders_booked.clear();
   list_hand_violation.clear();
   //borders.push_back(sb);
   redraw = true;
   first_init = true;
   safety_line_mask = cv::Mat::zeros(1024,1024,CV_8U);
   sf_line_colored = cv::Mat::zeros(1024,1024,CV_8UC3);
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

   std::cout<<"StaticBorderManager running\n";

}
//add a border to the manager. each border is added to an array.
//Then we split the borders between the display (where they should be projected) and the monitoring
void StaticBorderManager::addBorder(StaticBorder sb)
{
   std::cout<<"StaticBorderManager::addBorder\n";
   borders.push_back(sb);
   BorderContentStatus tmp;
   tmp.id = sb.getId();
   tmp.col = sb.getCol();
   tmp.row = sb.getRow();
   tmp.status = 0;
   tmp.center = sb.getCenter();
   float t = sb.getBorderDiagonal();
   tmp.safety_distance = t * safety_factor;
   tmp.side_distance = t / 1.98; // distance from center to one of the corner with small extra for robustness
   borders_status.push_back(tmp);
   borders_booked.push_back(tmp);
   bool added = false;
   //add border to list_proj. This array gather the borders to be displayed by zones.
   //One element gather all the borders of the same zone, so if there is 2 elements then that means there is to zones where to display them.
   //The Projection struct helps to manage them.
   if(list_proj.size() > 0)
   {
      for(Projection p : list_proj)
      {
         int res = p.zone.compare(tmp.id);
         if(res == 0)
         {
            p.img = p.img + sb.drawBorder().clone();
            p.mask = p.mask + sb.drawMask().clone();
            added = true;
         }
      }
      if(!added)
      {
         Projection np;
         np.zone = sb.getZone();
         np.img = sb.drawBorder().clone();
         np.mask = sb.drawMask().clone();
         list_proj.push_back(np);
      }
   }
   else
   {
      Projection np;
      np.zone = sb.getZone();
      np.img = sb.drawBorder().clone();
      np.mask = sb.drawMask().clone();
      std::cout<<"list_proj.push_back(np);\n";
      list_proj.push_back(np);
   }
   redraw = true;
}
//Book a robot border by its id
void StaticBorderManager::bookBorderRobot(std::string id)
{
   int tmp_col;
   int tmp_row;
   //get position of the booked border in the grid and book it
   for(int i = 0; i < borders_booked.size(); i++)
   {
      if(borders_booked[i].id.compare(id) == 0)
      {
         borders_booked[i].status = 1;
         tmp_col = borders_booked[i].col;
         tmp_row = borders_booked[i].row;
      }
   }
   //change color of border booked
   for(int i = 0; i < borders.size(); i++)
   {
      if(borders[i].getId().compare(id) == 0)
      {
         borders[i].changeBorderColor(stat_booked);
      }
   }
   //if we also book the adjacent border (if the robot is big enough to cross some of them while moving - to avoid trigger a violation of border on empty ones)
   if(adj)
   {
      std::vector<std::string> adj_bdrs = getAdjacentBorders(tmp_row,tmp_col);
      //book borders
      for(int i = 0; i < borders_booked.size(); i++)
      {
         for(std::string id_adj : adj_bdrs)
         {
            if(id_adj.compare(borders_booked[i].id) == 0)
            {
               borders_booked[i].status = 1;
            }
         }
      }
      //change borders color
      for(int i = 0; i < borders.size(); i++)
      {
         for(std::string id_adj : adj_bdrs)
         {
            if(id_adj.compare(borders[i].getId()) == 0)
            {
               borders[i].changeBorderColor(stat_booked);
            }
         }
      }
   }
   //update the color of the borders
   updateProjection();
}
//Book a border for the operator. It signals the operator that an object can be picked by using a different color.
void StaticBorderManager::bookBorderOperator(std::string id)
{
   std::cout<<"CODE booking robot border\n";
   for(int i = 0; i < borders_booked.size(); i++)
   {
      if(borders_booked[i].id.compare(id) == 0)
      {
         borders_booked[i].status = 2;
      }
   }
   for(int i = 0; i < borders.size(); i++)
   {
      if(borders[i].getId().compare(id) == 0)
      {
         borders[i].changeBorderColor(stat_operator);
      }
   }
   updateProjection();
}

//release a border booked by the robot
void StaticBorderManager::releaseRobotBorder(std::string id, int status)
{
   //release booking and change color
   std::cout<<"RELEASING BORDER\n";
   if(status == 0)
   {
      for(int i = 0; i < borders_booked.size(); i++)
      {
         if(borders_booked[i].id.compare(id) == 0)
         {
            borders_booked[i].status = 0;
         }
      }
      for(int i = 0; i < borders.size(); i++)
      {
         if(borders[i].getId().compare(id) == 0)
         {
            borders[i].changeBorderColor(stat_free);
         }
      }
   }
   else
   {
      for(int i = 0; i < borders_booked.size(); i++)
      {
         for(int j = 0; j < borders.size(); j++)
         {
            if(borders[j].getId().compare(borders_booked[i].id) == 0)
            {
               if(borders_booked[i].status == 1)
               {
                  borders[j].changeBorderColor(stat_free);
               }
            }
         }
      }
   }
   updateProjection();
}
//release a booking made by the operator
void StaticBorderManager::releaseOperatorBorder(std::string id, int status)
{
   //release booking and change color
   if(status == 0)
   {
      for(int i = 0; i < borders_booked.size(); i++)
      {
         if(borders_booked[i].id.compare(id) == 0)
         {
            borders_booked[i].status = 0;
         }
      }
      for(int i = 0; i < borders.size(); i++)
      {
         if(borders[i].getId().compare(id) == 0)
         {
            borders[i].changeBorderColor(stat_free);
         }
      }
   }
   else
   {
      for(int i = 0; i < borders_booked.size(); i++)
      {
         for(int j = 0; j < borders.size(); j++)
         {
            if(borders[j].getId().compare(borders_booked[i].id) == 0)
            {
               if(borders_booked[i].status == 2)
               {
                  borders[j].changeBorderColor(stat_free);
               }
            }
         }
      }
   }
   updateProjection();
}
//update projections. Ir redraws the borders with updated values from the array borders.
//so it basically modify the list of projection list_proj
void StaticBorderManager::updateProjection()
{
   list_proj.clear();
   bool first = true;
   for(StaticBorder sb : borders)
   {
      if(first)
      {
         Projection p;
         p.zone = sb.getZone();
         p.img = sb.drawBorder().clone();
         p.mask = sb.drawMask().clone();
         list_proj.push_back(p);
         first = false;
      }
      else
      {
         bool added = false;
         for(Projection p : list_proj)
         {
            if(p.zone == sb.getZone())
            {
               p.img = p.img + sb.drawBorder().clone();
               p.mask = p.mask + sb.drawMask().clone();
               added = true;
            }
         }
         if(added == false)
         {
            Projection p;
            p.zone = sb.getZone();
            p.img = sb.drawBorder().clone();
            p.mask = sb.drawMask().clone();
            list_proj.push_back(p);
         }
      }
   }
   redraw = true;
}
//Get the adjacent borders. This is when the adjacent option is true
std::vector<std::string> StaticBorderManager::getAdjacentBorders(int r, int c)
{
   std::vector<std::string> list_adj;
   list_adj.clear();
   if(s_rows == 1)
   {
      for(BorderContentStatus bdr : borders_booked)
      {
         if(bdr.col == c-1 || bdr.col == c+1)
         {
            list_adj.push_back(bdr.id);
         }
      }
   }
   else
   {
      for(BorderContentStatus bdr : borders_booked)
      {
         if(bdr.col == c-1 || bdr.col == c+1 || bdr.row == r-1 || bdr.row == r+1)
         {
            list_adj.push_back(bdr.id);
         }
      }
   }
   return list_adj;
}
//service that gives the occupancy of the borders
// If ther is an object inside a border, this one can't be booked
bool StaticBorderManager::getBordersService(integration::ListStaticBordersStatus::Request& req, integration::ListStaticBordersStatus::Response& res)
{
   std::cout<<"getBordersService border \n";
   res.status_borders.clear();
   getObjectsBorders(depth_map);
   for(BorderContentStatus bdr : borders_status)
   {
      integration::StaticBorderStatus sbs;
      sbs.id = bdr.id;
      sbs.status = bdr.status;
      std::cout<<"sbs.id: "<<sbs.id<<"\n";
      std::cout<<"sbs.status : "<<sbs.status<<"\n";
      res.status_borders.push_back(sbs);
   }

   return true;
}

//callback for depthmap, this is where the monitoring happens
void StaticBorderManager::depthMapCallback(const sensor_msgs::ImageConstPtr& msg_dm)
{
   ros::Time begin = ros::Time::now();
   sf_line = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   std_msgs::Header header;
   header.frame_id = msg_dm->header.frame_id;
   header.seq = msg_dm->header.seq;
   header.stamp = msg_dm->header.stamp;
   //if we must redraw the borders
   if(redraw)
   {
      safety_line_mask = cv::Mat::zeros(1024,1024,CV_8U);
      sf_line_colored = cv::Mat::zeros(1024,1024,CV_8UC3);
      for(Projection i : list_proj)
      {
         sf_line_colored = sf_line_colored + i.img.clone();
         safety_line_mask = safety_line_mask + i.mask.clone();
      }
      publishBorder();
      redraw = false;
      first_baseline = true;
      first_init = true;
   }
   cv::Mat res_bl = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   res_dm = cv::Mat(1024, 1024, CV_32F,cv::Scalar(std::numeric_limits<float>::min()));
   cv::Mat cluster_img = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   cv::Mat enhanced = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   bool is_cluster = false;
   cv::Mat res_mask_dm;
   cv::Mat res_mask_bl;
   cv::Mat res_enhanced;
   
   try
   {
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg_dm, sensor_msgs::image_encodings::TYPE_32FC1);
      depth_map = cv_ptr->image.clone();
      //detect border crossing by providing the mask of the borders
      std::vector<cv::KeyPoint> kpts = detectBorderCrossing(safety_line_mask);
      raiseBorderViolation(kpts,header);
      
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }
}

//throw a border violation if there is any crossing
//keypoints are the points where the crossing has been detected by change of depth value.
//Here, there is a clear violation if there is a change in depth AND if a hand has been detected nearby
void StaticBorderManager::raiseBorderViolation(std::vector<cv::KeyPoint>& keypoints, std_msgs::Header& header)
{
   for(cv::KeyPoint kp : keypoints)
   {
      for(BorderContentStatus bds : list_hand_violation)
      {
         cv::Point tmp_keypt;
         tmp_keypt.x = static_cast<int>(kp.pt.x);
         tmp_keypt.y = static_cast<int>(kp.pt.y);
         cv::Point bd_center;
         bd_center.x = static_cast<int>(bds.center.x);
         bd_center.y = static_cast<int>(bds.center.y);
         float dist = sqrt(pow(tmp_keypt.x - bd_center.x,2) + pow(tmp_keypt.y - bd_center.y,2));
         if(dist < bds.side_distance)
         {
            std::cout<<"violation border : "<<bds.col<<"\n";
            for(StaticBorder sb : borders)
            {
               if(sb.getId().compare(bds.id) == 0)
               {
                  geometry_msgs::PoseStamped pose_location;
                  pose_location.header = header;
                  pose_location.header.frame_id = "base";
                  pose_location.pose = sb.transformPtToRobotSpace(tmp_keypt.x,tmp_keypt.y);
                  integration::SafetyBorderViolation msg_border;
                  msg_border.header = header;
                  msg_border.header.frame_id = "base";
                  msg_border.request_id = sb.getId();
                  //get violated border
                  geometry_msgs::PolygonStamped bord = sb.getBorderRobotSpace();
                  bord.header = header;
                  bord.header.frame_id = "base";
                  msg_border.initial_border = bord;
                  msg_border.target_location = pose_location.pose;
                  pub_border_violation.publish(msg_border);
                  pub_border_polygon.publish(bord);
                  pub_pose_violation.publish(pose_location);
               }
            }
         }
      }
   }
}
//detect border crossing in depthmap
std::vector<cv::KeyPoint> StaticBorderManager::detectBorderCrossing(cv::Mat& mask)
{
   cv::Mat res_sub = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   cv::Mat res_enhanced = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   cv::Mat img_col = cv::Mat::zeros(1024,1024,CV_8UC1);
   cv::Mat img_blur = cv::Mat::zeros(1024,1024,CV_8UC1);
   cv::Mat img_contours = cv::Mat::zeros(1024,1024,CV_8UC3);
   cv::subtract(depth_map,baseline_dm,res_sub,mask);
   //enhamcing depth values because the difference can be very small
   res_enhanced = enhanceDepth(res_sub,0.01);
   res_enhanced.convertTo(img_col,CV_8UC1,255.0);
   cv::SimpleBlobDetector::Params params; 
   // Change thresholds
   params.minThreshold = 50;
   params.maxThreshold = 255;
   // Filter by Area.
   params.filterByArea = false;
   params.minArea = 5;
   // Filter by Circularity
   params.filterByCircularity = false;
   params.minCircularity = 0.1;
   // Filter by Convexity
   params.filterByConvexity = false;
   params.minConvexity = 0.87;
   // Filter by Inertia
   params.filterByInertia = true;
   params.minInertiaRatio = 0.01;
   cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params); 
   // Detect blobs.
   std::vector<cv::KeyPoint> keypoints;
   detector->detect(img_col, keypoints);
   baseline_dm.convertTo(img_col,CV_8UC1,255.0);
   //for display
   //cv::imshow(OPENCV_TEST, img_col);
   //cv::waitKey(1);
   return keypoints;
}
//coordinates of detected hands in depthmap
//if a hand location < safety_factor, then a hand is too close to a border
void StaticBorderManager::handTrackingCallback(const unity_msgs::poiPCLConstPtr& msg)
{
   list_hand_violation.clear();
   for(BorderContentStatus bdr : borders_booked)
   {
      if(bdr.status == 1)
      {
         cv::Point pt_center;
         pt_center.x = static_cast<int>(bdr.center.x);
         pt_center.y = static_cast<int>(bdr.center.y);
         for(int i = 0; i < msg->pts.size(); i++)
         {
            cv::Point pt_hand;
            pt_hand.x = static_cast<int>(msg->pts[i].x);
            pt_hand.y = static_cast<int>(msg->pts[i].y);
            float dist = sqrt(pow(pt_hand.x - pt_center.x,2) + pow(pt_hand.y - pt_center.y,2));
            if(dist < bdr.safety_distance)
            {
               //std::cout<<"hand violation detected on border : "<<bdr.col<<" with distance : "<<dist<<" compared to "<<bdr.safety_distance<<"\n";
               list_hand_violation.push_back(bdr);
            }
         }
      }
   }
}
// Check the depth inside the borders to detect if there are any objects
void StaticBorderManager::getObjectsBorders(cv::Mat& image)
{
   cv::Mat res_bl = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   cv::Mat enhanced = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   cv::Mat res_mask_dm;     
   cv::Mat res_mask_bl;
   cv::Mat res_enhanced;
   cv::Mat mask_detect = cv::Mat::zeros(1024,1024,CV_8U);
   getMaskDetection(mask_detect);
   image.copyTo(res_mask_dm,mask_detect);
   baseline_dm.copyTo(res_mask_bl,mask_detect);
   cv::subtract(res_mask_dm, res_mask_bl,res_bl,mask_detect);
   res_enhanced = enhanceDepth(res_bl,0);
   fillOccupancy(res_enhanced);
   //cv::imshow(OPENCV_TEST, res_enhanced);
   //cv::waitKey(1);
}
//fill the borders occupancy
void StaticBorderManager::fillOccupancy(cv::Mat& image)
{
   for(int i = 0; i < borders_status.size(); i++)
   {
      bool cluster = isClusterInsideBorder(image,borders_status[i]);
      if(cluster)
      {
         borders_status[i].status = 1;
      }
      else
      {
         if(borders_booked[i].status == 1)
         {
            borders_status[i].status = 1;
         }
         else
         {
            borders_status[i].status = 0;
         }
      }
   }
}
//draw a mask (circle) inside the border that is used to monitor is there are objects there
void StaticBorderManager::getMaskDetection(cv::Mat& image)
{
   for(int i = 0; i < borders.size(); i++)
   {
      cv::Point tmp;
      geometry_msgs::Point c = borders[i].getCenter();
      tmp.x = static_cast<int>(c.x);
      tmp.y = static_cast<int>(c.y);
      circle(image, tmp,15, cv::Scalar(255, 255, 255), -1);
   }
}
//read a depthimage. here it's the baseline image generated during calibration
bool StaticBorderManager::readRawImage(cv::Mat& image, const std::string& filename)
{
   int rows, cols, data, depth, type, channels;
   ifstream file (filename, ios::in|ios::binary);
   if (!file.is_open())
      return false;
   try {
      file.read(reinterpret_cast<char *>(&rows), sizeof(rows));
      file.read(reinterpret_cast<char *>(&cols), sizeof(cols));
      file.read(reinterpret_cast<char *>(&depth), sizeof(depth));
      file.read(reinterpret_cast<char *>(&type), sizeof(type));
      file.read(reinterpret_cast<char *>(&channels), sizeof(channels));
      file.read(reinterpret_cast<char *>(&data), sizeof(data));
      image = cv::Mat(rows, cols, type);
      file.read(reinterpret_cast<char *>(image.data), data);
   } catch (...) {
      file.close();
      return false;
   }

   file.close();
   return true;
}
//check if there is a cluster inside the borders. Used to detect if ther is an object
bool StaticBorderManager::isClusterInsideBorder(cv::Mat& image, BorderContentStatus bdr)
{
   bool result = false;
   cv::Point tmp;
   int count = 0;
   tmp.x = static_cast<int>(bdr.center.x);
   tmp.y = static_cast<int>(bdr.center.y);
   for(int i = tmp.y-3; i < tmp.y+3; i++)
   {
      for(int j = tmp.x-3; j < tmp.x+3; j++)
      {
         float val = image.at<float>(i,j);
         if(val > 0.1)
         {
            count++;
         }
      }
   }
   if(count > 12)
   {
      result = true;
   }

   return result;
}
//enhaced depth of depth image. After filtering, the difference between baseline and depthmap can be very small,
// so we multiply the depth by a factor to be able to see the changes
cv::Mat StaticBorderManager::enhanceDepth(cv::Mat img, float thr)
{
   for(int i = 0; i < img.rows; i++)
   {
      for(int j = 0; j < img.cols; j++)
      {
         float tmp = img.at<float>(i,j);
         if(tmp > thr)
         {
            img.at<float>(i,j) = tmp * 100;
         }
         else
         {
            img.at<float>(i,j) = 0.0;
         }
      }
   }

   return img;
}
//publish the border to project them
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
