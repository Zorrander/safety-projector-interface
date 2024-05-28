/*
Class that manage static borders. It display and monitors them.
*/
#include "border/StaticBorderManager.h"


// Params are almost the same as in StaticBorder
//adjacent : if true then adjacent borders must be booked too
//sf_factor : safety_factor for the hand detection.
StaticBorderManager::StaticBorderManager(ros::NodeHandle *nh_, int rows, int cols, float sf_factor, bool adjacent, std_msgs::ColorRGBA status_booked, std_msgs::ColorRGBA status_free, std_msgs::ColorRGBA status_operator):
nh(*nh_),
it_(nh),
s_rows(rows),
s_cols(cols),
safety_factor(sf_factor),
adj(adjacent),
stat_booked(status_booked),
stat_free(status_free),
stat_operator(status_operator)
{
   service_borders = nh.advertiseService("/execution/projector_interface/integration/services/list_static_border_status", &StaticBorderManager::getBordersService, this);
   dm_sub_ = it_.subscribe("/detection/depth_map", 1,&StaticBorderManager::depthMapCallback, this);  
   sub_hand_detection = nh.subscribe("/hand_tracking/dm/coordinates", 1,&StaticBorderManager::handTrackingCallback, this);
   pub_border_projection = nh.advertise<unity_msgs::LBorder>("/projector_interface/display_static_border",1);
   pub_border_violation = nh.advertise<integration::SafetyBorderViolation>("/execution/projector_interface/integration/topics/safety_border_violation",1);
   pub_pose_violation = nh.advertise<geometry_msgs::PoseStamped>("/projector_interface/pose_violation",1);
   pub_border_polygon = nh.advertise<geometry_msgs::PolygonStamped>("/border_violated",1);

   ros::param::get("calibration_folder", calibration_folder);
   name_bl = calibration_folder + "baseline";
   baseline_dm = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
   readRawImage(baseline_dm, name_bl);

   redraw = true;

   list_proj.clear();
   borders.clear();
   borders_status.clear();
   borders_booked.clear();
   list_hand_violation.clear();

   safety_line_mask = cv::Mat::zeros(1024,1024,CV_8U);
   depth_map = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));

   // Create a mask to detect objects within the borders
   mask_detect = cv::Mat::zeros(1024,1024,CV_8U);
   getMaskDetection(mask_detect);

   ROS_INFO("StaticBorderManager running");
}

void StaticBorderManager::addBorder(StaticBorder sb) {
    std::cout<<"StaticBorderManager::addBorder\n";
    // Add border to the list of borders
    borders.push_back(sb);

    // Initialize BorderContentStatus for the new border
    BorderContentStatus tmp;
    tmp.id = sb.getId();
    tmp.col = sb.getCol();
    tmp.row = sb.getRow();
    tmp.status = 0;
    tmp.center = sb.getCenter();
    float t = sb.getBorderDiagonal();
    tmp.safety_distance = t * safety_factor;
    tmp.side_distance = t / 1.98; // distance from center to one of the corners with a small extra for robustness

    // Add BorderContentStatus to both status and booked lists
    borders_status.push_back(tmp);
    borders_booked.push_back(tmp);

    // Update list_proj for display
    // Check if the zone of the new border already exists in list_proj
    bool zoneExists = false;
    for (auto& p : list_proj) {
        if (p.zone == sb.getZone()) {
            p.img += sb.drawBorder().clone();
            p.mask += sb.drawMask().clone();
            zoneExists = true;
            break;
        }
    }

    // If the zone doesn't exist, create a new Projection object
    if (!zoneExists) {
        Projection np;
        np.zone = sb.getZone();
        np.img = sb.drawBorder().clone();
        np.mask = sb.drawMask().clone();
        list_proj.push_back(np);
    }
    redraw = true;
}

//Book a robot border by its id
void StaticBorderManager::bookBorderRobot(std::string id)
{
   ROS_INFO("BOOKING BORDER %s", id.c_str());
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
   for(BorderContentStatus bdr : borders_status)
   {
      ROS_INFO("Border %s --> %d", bdr.id.c_str(), bdr.status);
   }
}

//Book a border for the operator. It signals the operator that an object can be picked by using a different color.
void StaticBorderManager::bookBorderOperator(std::string id)
{
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
}

//release a border booked by the robot
void StaticBorderManager::releaseRobotBorder(std::string id, int status)
{
   //release booking and change color
   ROS_INFO("RELEASING BORDER %s -> %i", id.c_str(), status);
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
                  if(borders_booked[j].id.compare(id) != 0)
                  {
                     borders_booked[j].status = 0;
                  }
               }

            }
         }
      }
   }
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
}

/**
 * Get the IDs of adjacent borders based on row and column indices.
 * 
 * @param row The row index.
 * @param col The column index.
 * @return A vector of IDs of adjacent borders.
 */
std::vector<std::string> StaticBorderManager::getAdjacentBorders(int row, int col)
{
   std::vector<std::string> list_adj;
   
   // Check if single row or multiple rows
   for (const BorderContentStatus& bdr : borders_booked)
   {
      if ((s_rows == 1 && (bdr.col == col - 1 || bdr.col == col + 1)) ||
          (s_rows > 1 && (bdr.col == col - 1 || bdr.col == col + 1 || 
                         bdr.row == row - 1 || bdr.row == row + 1)))
      {
         list_adj.push_back(bdr.id);
      }
   }
   
   return list_adj;
}


void StaticBorderManager::depthMapCallback(const sensor_msgs::ImageConstPtr& msg_dm)
{
   std_msgs::Header& header = const_cast<std_msgs::Header&>(msg_dm->header);
   try
   {
      // Convert ROS message to OpenCV image
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(msg_dm, sensor_msgs::image_encodings::TYPE_32FC1);
      depth_map = cv_ptr->image.clone();

      if(redraw)
      {
         safety_line_mask = cv::Mat::zeros(1024,1024,CV_8U);
         for(Projection i : list_proj)
         {
            safety_line_mask = safety_line_mask + i.mask.clone();
         }
         redraw = false;
      }
      // Detect border crossings and raise violations
      std::vector<cv::KeyPoint> key_points = detectBorderCrossing(safety_line_mask);
      raiseBorderViolation(key_points, header);
   }
   catch (const cv_bridge::Exception& e)
   {
      ROS_ERROR("cv_bridge exception: %s", e.what());
   }
   catch (const std::exception& ex)
   {
      ROS_ERROR("Exception: %s", ex.what());
   }
}

void StaticBorderManager::handTrackingCallback(const unity_msgs::poiPCLConstPtr& msg)
{
   list_hand_violation.clear();

   for (const auto& booked_border : borders_booked)
   {
      if (booked_border.status == 1)
      {
         const cv::Point border_center(static_cast<int>(booked_border.center.x), static_cast<int>(booked_border.center.y));

         for (const auto& hand_point : msg->pts)
         {
            const cv::Point hand_position(static_cast<int>(hand_point.x), static_cast<int>(hand_point.y));
            const float distance = cv::norm(hand_position - border_center);

            if (distance < booked_border.safety_distance)
            {
               // Hand violation detected
               list_hand_violation.push_back(booked_border);
               // No need to check further once a violation is detected
               break;
            }
         }
      }
   }
}


void StaticBorderManager::raiseBorderViolation(std::vector<cv::KeyPoint>& keypoints, std_msgs::Header& header)
{
   for (const auto& keypoint : keypoints)
   {
      for (const auto& border : list_hand_violation)
      {
         const cv::Point tmp_keypoint(static_cast<int>(keypoint.pt.x), static_cast<int>(keypoint.pt.y));
         const cv::Point border_center(static_cast<int>(border.center.x), static_cast<int>(border.center.y));
         const float distance = cv::norm(tmp_keypoint - border_center);

         if (distance < border.side_distance)
         {
            std::cout << "Violation border: " << border.col << "\n";

            for (auto& sb : borders)
            {
               if (sb.getId() == border.id)
               {
                  geometry_msgs::PoseStamped pose_location;
                  pose_location.header = header;
                  pose_location.header.frame_id = "base";
                  pose_location.pose = sb.transformPtToRobotSpace(tmp_keypoint.x, tmp_keypoint.y);

                  integration::SafetyBorderViolation msg_border;
                  msg_border.header = header;
                  msg_border.header.frame_id = "base";
                  msg_border.request_id = sb.getId();
                  
                  // Get violated border
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


std::vector<cv::KeyPoint> StaticBorderManager::detectBorderCrossing(cv::Mat& mask)
{
   // Check if the mask is valid
   if (mask.empty() || mask.size() != cv::Size(1024, 1024))
   {
      ROS_ERROR("Invalid mask provided for border crossing detection.");
      return std::vector<cv::KeyPoint>();
   }

   cv::Mat res_sub = cv::Mat(1024, 1024, CV_32FC1, cv::Scalar(std::numeric_limits<float>::min()));
   cv::Mat res_enhanced = cv::Mat(1024, 1024, CV_32FC1, cv::Scalar(std::numeric_limits<float>::min()));
   cv::Mat img_col, img_blur, img_contours;

   // Subtract baseline from depth map using the mask
   cv::subtract(depth_map, baseline_dm, res_sub, mask);

   // Enhance depth values
   res_enhanced = enhanceDepth(res_sub, 0.01);
   res_enhanced.convertTo(img_col, CV_8UC1, 255.0);

   // Configure blob detector parameters
   cv::SimpleBlobDetector::Params params;
   params.minThreshold = 50;
   params.maxThreshold = 255;
   params.filterByArea = false;
   params.minArea = 5;
   params.filterByCircularity = false;
   params.minCircularity = 0.1;
   params.filterByConvexity = false;
   params.minConvexity = 0.87;
   params.filterByInertia = true;
   params.minInertiaRatio = 0.01;

   // Create blob detector
   cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params); 

   // Detect blobs
   std::vector<cv::KeyPoint> keypoints;
   detector->detect(img_col, keypoints);

   // Convert baseline depth map for display
   baseline_dm.convertTo(img_col, CV_8UC1, 255.0);

   // Return detected keypoints
   return keypoints;
}


bool StaticBorderManager::getBordersService(integration::ListStaticBordersStatus::Request& req, integration::ListStaticBordersStatus::Response& res)
{
   ROS_INFO("Checking border status...");
   res.status_borders.clear();

   // Update the status of borders
   updateBorderStatus();

   // Populate the response with border statuses
   for (const auto& border_status : borders_status)
   {
      integration::StaticBorderStatus sbs;
      sbs.id = border_status.id;
      sbs.status = border_status.status;
      res.status_borders.push_back(sbs);
      ROS_INFO("Border %s --> %d", sbs.id.c_str(), sbs.status);
   }

   ROS_INFO("Border status check complete.");
   return true;
}

// Update the status of borders based on object occupancy
void StaticBorderManager::updateBorderStatus()
{
   ROS_INFO("Looking for objects within the borders...");   

   // Get the depth information within the borders
   cv::Mat depth_within_borders;
   getObjectsDepth(depth_within_borders, mask_detect);

   // Enhance the depth information
   cv::Mat enhanced_depth = enhanceDepth(depth_within_borders, 0);

   // Update border statuses based on enhanced depth information
   for (int i = 0; i < borders.size(); ++i)
   {
      // Check if there is a cluster inside the border
      bool cluster_inside_border = isClusterInsideBorder(enhanced_depth, borders_status[i]);
      
      // Update the status accordingly
      if (cluster_inside_border || borders_booked[i].status == 1)
      {
         borders_status[i].status = 1;
         ROS_INFO("Border %i status updated: 1", i); 
      }
      else
      {
         borders_status[i].status = 0;
         ROS_INFO("Border %i status updated: 0", i); 
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
      ROS_INFO("Detected cluster inside border");
      result = true;
   }

   return result;
}

// Get the depth information within the borders
void StaticBorderManager::getObjectsDepth(cv::Mat& depth_within_borders, const cv::Mat& mask_detect)
{
   cv::Mat res_mask_dm, res_mask_bl, res_bl;

   // Apply the mask to the depth image
   depth_map.copyTo(res_mask_dm, mask_detect);
   baseline_dm.copyTo(res_mask_bl, mask_detect);

   // Calculate the depth difference within the borders
   cv::subtract(res_mask_dm, res_mask_bl, res_bl, mask_detect);
   
   depth_within_borders = res_bl.clone();
}

//enhaced depth of depth image. After filtering, the difference between baseline and depthmap can be very small,
// so we multiply the depth by a factor to be able to see the changes
cv::Mat StaticBorderManager::enhanceDepth(cv::Mat img, float thr)
{
   cv::Mat enhanced_img = img.clone();
   for(int i = 0; i < enhanced_img.rows; i++)
   {
      for(int j = 0; j < enhanced_img.cols; j++)
      {
         float tmp = enhanced_img.at<float>(i,j);
         if(tmp > thr)
         {
            enhanced_img.at<float>(i,j) = tmp * 100;
         }
         else
         {
            enhanced_img.at<float>(i,j) = 0.0;
         }
      }
   }

   return enhanced_img;
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


void StaticBorderManager::updateProjection()
{
   // Clear the list of projections
   list_proj.clear();

   // Iterate through each border
   for(StaticBorder& sb : borders)
   {
      bool found = false;

      // Check if the border's zone is already in the list of projections
      for(Projection& p : list_proj)
      {
         if(p.zone == sb.getZone())
         {
            // Update the projection with the border's border and mask
            p.img += sb.drawBorder().clone();
            p.mask += sb.drawMask().clone();
            found = true;
            break;
         }
      }

      // If the zone was not found, add a new projection
      if(!found)
      {
         Projection p;
         p.zone = sb.getZone();
         p.img = sb.drawBorder().clone();
         p.mask = sb.drawMask().clone();
         list_proj.push_back(p);
      }
   }
   redraw = true;
}

//publish the border to project them
void StaticBorderManager::publishBorder()
{
   updateProjection();

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
