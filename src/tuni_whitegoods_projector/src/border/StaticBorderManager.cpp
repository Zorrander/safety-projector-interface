/*
Class that manage static borders. It display and monitors them.
*/
#include "border/StaticBorderManager.h"

#include "tuni_whitegoods_msgs/TransformRobotCameraCoordinates.h"
#include "tuni_whitegoods_msgs/Transform3DToPixel.h"
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
dist_buffers(1000.0, 1000.0),
stat_operator(status_operator),
client_button_color("execution/projector_interface/integration/actions/set_virtual_button_change_color", true),
tfBuffer(),  // Initialize tfBuffer
tfListener(new tf2_ros::TransformListener(tfBuffer))  // Initialize tfListener with tfBuffer
{
   client = nh_->serviceClient<tuni_whitegoods_msgs::TransformRobotCameraCoordinates>("transform_world_coordinates_frame");
   client_3D_to_pixel = nh_->serviceClient<tuni_whitegoods_msgs::Transform3DToPixel>("transform_3D_to_pixel");

   service_borders = nh.advertiseService("/execution/projector_interface/integration/services/list_static_border_status", &StaticBorderManager::getBordersService, this); 
   sub_hand_detection = nh.subscribe("/hand_tracking/dm/coordinates", 1,&StaticBorderManager::handTrackingCallback, this);
   pub_border_projection = nh.advertise<unity_msgs::LBorder>("/projector_interface/display_static_border",1);
   pub_border_violation = nh.advertise<integration::SafetyBorderViolation>("/execution/projector_interface/integration/topics/safety_border_violation",1);
   pub_pose_violation = nh.advertise<geometry_msgs::PoseStamped>("/projector_interface/pose_violation",1);
   
   vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

   pub_event = nh.advertise<integration::VirtualButtonEventArray> ("/execution/projector_interface/integration/topics/virtual_button_event_array", 1);
   
   client_button_color.waitForServer();
   sub_poi = nh.subscribe("/depth_interface/poi_depthmap", 1, &StaticBorderManager::pointsOfInterestCb,this);

   depth_sub = nh_->subscribe("/depth_to_rgb/image_raw", 1, &StaticBorderManager::depthImageCallback,this);
   object_detection_pub = it_.advertise("odin/visualization/object_detection", 1);

   border_crossed = false;
   button_pressed = false; 

   // Enable dedicated thread
   tfBuffer.setUsingDedicatedThread(true);

   geometry_msgs::TransformStamped transformStamped;

   // Try for a limited number of times
   for (int i = 0; i < 10; ++i) {
       try {
            transformStamped = tfBuffer.lookupTransform("base", "rgb_camera_link", ros::Time(0));
            ROS_INFO("Got transform: translation (%.2f, %.2f, %.2f), rotation (%.2f, %.2f, %.2f, %.2f)",
                     transformStamped.transform.translation.x,
                     transformStamped.transform.translation.y,
                     transformStamped.transform.translation.z,
                     transformStamped.transform.rotation.x,
                     transformStamped.transform.rotation.y,
                     transformStamped.transform.rotation.z,
                     transformStamped.transform.rotation.w);
            break;
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF2 Transform Exception: %s", ex.what());
        }
        ros::Duration(1.0).sleep(); // Wait a bit before checking again
   }
   ros::Duration(5.0).sleep(); 
   ROS_INFO("StaticBorderManager running");
}

void StaticBorderManager::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg){
   cv_bridge::CvImagePtr cv_bridge_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
   cv_depth = cv_bridge_depth->image.clone();

   // Normalize the depth map to the range 0-255 for better visualization
   cv::Mat depth_normalized;
   cv::normalize(cv_depth, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
   // Apply a color map
   cv::Mat depth_colormap;
   cv::applyColorMap(depth_normalized, depth_colormap, cv::COLORMAP_JET);
      
   for(const auto& sb : borders)
   {  
      roi_rect = cv::Rect(sb->top_left_cam_point, sb->bottom_right_cam_point);

      cv::Mat roi_depthmap = cv_depth(roi_rect);
      // Apply the depth range threshold
      cv::Mat mask;
      cv::Scalar min_depth = cv::Scalar(1250); // TO BE ADJUSTED
      cv::Scalar max_depth = cv::Scalar(1300); // TO BE ADJUSTED
      cv::inRange(roi_depthmap, min_depth, max_depth, mask);

      // Find contours in the mask
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      // Draw contours 
      cv::Scalar color(255, 255, 255); 
      cv::drawContours(depth_colormap, contours, -1, color, 2);
      
      // Draw ROI
      cv::rectangle(depth_colormap, roi_rect, cv::Scalar(255,255,255), 3, cv::LINE_8);

      // Check if any contour is found
      if (contours.size() > 0){
         sb->changeThickness(-1);
         sb->occupied = true;       
      } else {
         sb->occupied = false;
      }
   }

   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depth_colormap).toImageMsg();
   object_detection_pub.publish(msg);

}

void StaticBorderManager::addBorder(std::shared_ptr<StaticBorder> sb) {
    std::cout<<"StaticBorderManager::addBorder\n";

    // Initialize BorderContentStatus for the new border
    BorderContentStatus tmp;
    tmp.id = sb->getId();
    tmp.col = sb->getCol();
    tmp.row = sb->getRow();
    tmp.status = 0;
    tmp.center = sb->getCenter();
    float t = sb->getBorderDiagonal();
    tmp.safety_distance = t * safety_factor;
    tmp.side_distance = t / 1.98; // distance from center to one of the corners with a small extra for robustness

    // Add BorderContentStatus to both status and booked lists
    borders_status.push_back(tmp);
    borders_booked.push_back(tmp);
   
   geometry_msgs::Point topLeftCornerPt;
   topLeftCornerPt.x = sb->border_robot_space.polygon.points[0].x; 
   topLeftCornerPt.y = sb->border_robot_space.polygon.points[0].y;

   geometry_msgs::Point topRightCornerPt;
   topRightCornerPt.x = sb->border_robot_space.polygon.points[0].x; 
   topRightCornerPt.y = sb->border_robot_space.polygon.points[1].y;

   geometry_msgs::Point bottomRightCornerPt;
   bottomRightCornerPt.x = sb->border_robot_space.polygon.points[1].x; 
   bottomRightCornerPt.y = sb->border_robot_space.polygon.points[1].y;

   geometry_msgs::Point bottomLeftCornerPt;
   bottomLeftCornerPt.x = sb->border_robot_space.polygon.points[1].x; 
   bottomLeftCornerPt.y = sb->border_robot_space.polygon.points[0].y;

   // Create Rviz marker 
   visualization_msgs::Marker marker;
   marker.header.frame_id = "base";
   marker.header.stamp = ros::Time::now();
   marker.id = borders.size()+10;;
   marker.type = visualization_msgs::Marker::LINE_STRIP;
   marker.action = visualization_msgs::Marker::ADD;
   marker.scale.x = 0.01;  // Line width
   marker.color.r = 0.0;
   marker.color.g = 1.0;
   marker.color.b = 0.0;
   marker.color.a = 1.0;

   marker.points.push_back(topLeftCornerPt);
   marker.points.push_back(topRightCornerPt);
   marker.points.push_back(bottomRightCornerPt);
   marker.points.push_back(bottomLeftCornerPt);
   marker.points.push_back(topLeftCornerPt);

   vis_pub.publish( marker );

    geometry_msgs::PoseStamped in_point_stamped;
    tuni_whitegoods_msgs::TransformRobotCameraCoordinates srv;
    tuni_whitegoods_msgs::Transform3DToPixel srv_3D_to_pixel;

    in_point_stamped.header.frame_id = "base";
    in_point_stamped.header.stamp = ros::Time(0);
    in_point_stamped.pose.position.x = topLeftCornerPt.x;
    in_point_stamped.pose.position.y = topLeftCornerPt.y;
   
    srv.request.in_point_stamped = in_point_stamped;
    srv.request.target_frame = "rgb_camera_link";
    client.call(srv);
       
    // Project to 2D image coordinates 
    srv_3D_to_pixel.request.x = srv.response.out_point_stamped.pose.position.x;
    srv_3D_to_pixel.request.y = srv.response.out_point_stamped.pose.position.y;
    srv_3D_to_pixel.request.z = 1.311792;
    client_3D_to_pixel.call(srv_3D_to_pixel);
    
    sb->top_left_cam_point.x = srv_3D_to_pixel.response.u;
    sb->top_left_cam_point.y = srv_3D_to_pixel.response.v;


   in_point_stamped.header.frame_id = "base";
   in_point_stamped.header.stamp = ros::Time(0);
   in_point_stamped.pose.position.x = bottomRightCornerPt.x;
   in_point_stamped.pose.position.y = bottomRightCornerPt.y;

   srv.request.in_point_stamped = in_point_stamped;
   srv.request.target_frame = "rgb_camera_link";
   client.call(srv);
   // Project to 2D image coordinates 
   srv_3D_to_pixel.request.x = srv.response.out_point_stamped.pose.position.x;
   srv_3D_to_pixel.request.y = srv.response.out_point_stamped.pose.position.y;
   srv_3D_to_pixel.request.z = 1.311792;
   client_3D_to_pixel.call(srv_3D_to_pixel);
   sb->bottom_right_cam_point.x = srv_3D_to_pixel.response.u;
   sb->bottom_right_cam_point.y = srv_3D_to_pixel.response.v;


    // Update list_proj for display
    // Check if the zone of the new border already exists in list_proj
    bool zoneExists = false;
    for (auto& p : list_proj) {
        if (p.zone == sb->getZone()) {
            p.img += sb->drawBorder().clone();
            zoneExists = true;
            break;
        }
    }

    // If the zone doesn't exist, create a new Projection object
    if (!zoneExists) {
        Projection np;
        np.zone = sb->getZone();
        np.img = sb->drawBorder().clone();
        list_proj.push_back(np);
    }

   // Add border to the list of borders
   borders.push_back(sb);
}

//Book a robot border by its id
void StaticBorderManager::bookBorderRobot(std::string id)
{
   ROS_INFO("BOOKING BORDER %s", id.c_str());
   int tmp_col;
   int tmp_row;

   for(int i = 0; i < borders.size(); i++)
   {
      if(borders[i]->getId().compare(id) == 0)
      {
         borders[i]->book();
         borders[i]->changeBorderColor(stat_booked);
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
            if(id_adj.compare(borders[i]->getId()) == 0)
            {
               borders[i]->changeBorderColor(stat_booked);
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
      if(borders[i]->getId().compare(id) == 0)
      {
         borders[i]->changeBorderColor(stat_operator);
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
      for(int i = 0; i < borders.size(); i++)
      {
         if(borders[i]->getId().compare(id) == 0)
         {
            borders[i]->release();
            borders[i]->changeBorderColor(stat_free);
         }
      }
   }
   else
   {
      for(int i = 0; i < borders_booked.size(); i++)
      {
         for(int j = 0; j < borders.size(); j++)
         {
            if(borders[j]->getId().compare(borders_booked[i].id) == 0)
            {
               if(borders_booked[i].status == 1)
               {
                  borders[j]->release();
                  borders[j]->changeBorderColor(stat_free);
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
         if(borders[i]->getId().compare(id) == 0)
         {
            borders[i]->changeBorderColor(stat_free);
         }
      }
   }
   else
   {
      for(int i = 0; i < borders_booked.size(); i++)
      {
         for(int j = 0; j < borders.size(); j++)
         {
            if(borders[j]->getId().compare(borders_booked[i].id) == 0)
            {
               if(borders_booked[i].status == 2)
               {
                  borders[j]->changeBorderColor(stat_free);
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


void StaticBorderManager::pointsOfInterestCb(const unity_msgs::InterfacePOIConstPtr& msg){
   // buttons.clear();
   for (unity_msgs::ElementUI btn: msg->poi){
      if (std::find(buttons.begin(), buttons.end(), btn) == buttons.end()) {
         Button b;
         b.id = btn.id;
         b.center = btn.elem; 
         buttons.push_back(b);
      } 
   }
}

void StaticBorderManager::handTrackingCallback(const unity_msgs::poiPCLConstPtr& msg)
{
   border_crossed = false;
   button_pressed = false;
   list_hand_violation.clear();
   std::string handType = msg->header.frame_id;
   for (auto& border : borders){
      geometry_msgs::Point border_center = border->getCenter();
      const cv::Point border_center(static_cast<int>(ros_border_center.x), static_cast<int>(ros_border_center.y));
      ROS_INFO("Border center x,y coordinates: (%i, %i)", border_center.x, border_center.y);
      for (const auto& hand_point : msg->pts)
      {
          geometry_msgs::PoseStamped hand_pose;
          hand_pose.pose.position.x = hand_point.x;
          hand_pose.pose.position.y = hand_point.y;

         if (handType == "Left") {
            cv::Point left_hand_position(static_cast<int>(hand_point.x), static_cast<int>(hand_point.y));
            ROS_INFO("hand_position x,y coordinates: (%i, %i)", left_hand_position.x, left_hand_position.y);

            float distance = cv::norm(left_hand_position - border_center);

            ROS_INFO("distance: (%f) | prev distance (%f) | difference between the two = (%f) vs diagonal (%f)", distance, dist_buffers[0], dist_buffers[0]-distance, border->getBorderDiagonal());
            if (distance < border->getBorderDiagonal())
            {
               // Hand violation detected
               border_crossed = true;
               border->left_hand_crossed = true;
               // No need to check further once a violation is detected
               break;
            }

            dist_buffers[0] = distance ; 
         }

         else if (handType == "Right") {
            cv::Point right_hand_position(static_cast<int>(hand_point.x), static_cast<int>(hand_point.y));
            ROS_INFO("hand_position x,y coordinates: (%i, %i)", right_hand_position.x, right_hand_position.y);

            float distance = cv::norm(right_hand_position - border_center);

            ROS_INFO("distance: (%f) | prev distance (%f) | difference between the two = (%f) vs diagonal (%f)", distance, dist_buffers[1], dist_buffers[1]-distance, border->getBorderDiagonal());
            if (distance < border->getBorderDiagonal())
            {
               // Hand violation detected
               border_crossed = true;
               border->right_hand_crossed = true; 
            }

            dist_buffers[1] = distance ;            
         }

         if (border_crossed){
            // No need to check further once a violation is detected
            break;
         }

      }

      if (border_crossed == false){
         if (handType == "Left"){
            border->left_hand_crossed = false;
         }
         else if (handType == "Right"){
            border->right_hand_crossed = false;
         }
      }
      
      if (border->right_hand_crossed || border->left_hand_crossed == true){
         if (border->occupied){
            border->changeBorderColor(stat_operator);
         }
         else if (border->booked){
            border->changeThickness(-1);
            //Communicate with openflow
            //pub_border_violation.publish(msg_border);
            //pub_border_polygon.publish(bord);
            //pub_pose_violation.publish(pose_location);
         } else {
            border->changeThickness(2);
         }   
      }

      else {
         border->changeThickness(1);
         if (border->booked){
            border->changeBorderColor(stat_booked);
         }
         else {
            border->changeBorderColor(stat_free);
         }
      }
      publishBorder();

   }


   for (auto& btn : buttons){
      geometry_msgs::Point ros_btn_center = btn.center;
      const cv::Point btn_center(static_cast<int>(ros_btn_center.x), static_cast<int>(ros_btn_center.y));
      // ROS_INFO("Button center x,y coordinates: (%i, %i)", btn_center.x, btn_center.y);
      for (const auto& hand_point : msg->pts)
      {
         if (handType == "Left") {
            cv::Point left_hand_position(static_cast<int>(hand_point.x), static_cast<int>(hand_point.y));
            ROS_INFO("hand_position x,y coordinates: (%i, %i)", left_hand_position.x, left_hand_position.y);

            float distance = cv::norm(left_hand_position - btn_center);
            if (distance < 10)
            {
               ROS_INFO("distance: (%f)", distance);
               ROS_INFO("onHover");
               // Hand violation detected
               button_pressed = true;
               btn.left_hand_hover = true;
               // No need to check further once a violation is detected
               break;
            }
         }

         else if (handType == "Right") {
            cv::Point right_hand_position(static_cast<int>(hand_point.x), static_cast<int>(hand_point.y));
            ROS_INFO("hand_position x,y coordinates: (%i, %i)", right_hand_position.x, right_hand_position.y);

            float distance = cv::norm(right_hand_position - btn_center);
            if (distance < 10)
            {
               ROS_INFO("distance: (%f)", distance);
               ROS_INFO("onHover");
               // Hand violation detected
               button_pressed = true;
               btn.right_hand_hover = true;
               // No need to check further once a violation is detected
               break;
            }

         }

         if (button_pressed){
            // No need to check further once a violation is detected
            break;
         }

      }

      if (button_pressed == false){
         if (handType == "Left"){
            btn.left_hand_hover = false;
         }
         else if (handType == "Right"){
            btn.right_hand_hover = false;
         }
      }
      
      if (btn.right_hand_hover || btn.left_hand_hover == true){
         ROS_INFO("BUTTON PRESSED");

         integration::SetVirtualButtonChangeColorGoal color_goal;
         color_goal.request_id = "go_button_color";
         color_goal.resource_id = "40";
         color_goal.button_color.r = 1.0;
         color_goal.button_color.g = 1.0;
         color_goal.button_color.b = 0.0;
         color_goal.button_color.a = 0.0;
         // ROS_INFO("new color (r: %f, g: %f, b: %f)", color_goal.button_color.r, color_goal.button_color.g, color_goal.button_color.b);
         client_button_color.sendGoal(color_goal);

         integration::VirtualButtonEvent btn_event;
         btn_event.virtual_button_id = btn.id;
         btn_event.event_type = btn_event.PRESSED;

         integration::VirtualButtonEventArray btn_events;
         btn_events.virtual_button_events.push_back(btn_event);

         pub_event.publish(btn_events);

      }

      else {
            ROS_INFO("BUTTON NOT PRESSED");
            integration::SetVirtualButtonChangeColorGoal color_goal;
            color_goal.request_id = "go_button_color";
            color_goal.resource_id = "40";
            color_goal.button_color.r = 0.0;
            color_goal.button_color.g = 0.0;
            color_goal.button_color.b = 1.0;
            color_goal.button_color.a = 0.0;
            client_button_color.sendGoal(color_goal);
         }
   }

   

   // Would be better to iterate through every border
   for (const auto& booked_border : borders_booked)
   {
      if (booked_border.status == 1)
      {
         const cv::Point border_center(static_cast<int>(booked_border.center.x), static_cast<int>(booked_border.center.y));
         for (const auto& hand_point : msg->pts)
         {
            const cv::Point hand_position(static_cast<int>(hand_point.x), static_cast<int>(hand_point.y));
            const float distance = cv::norm(hand_position - border_center);

            if (distance < booked_border.safety_distance*0.75)
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
   //getMaskDetection(mask_detect);

}


void StaticBorderManager::updateProjection()
{

   // Clear the list of projections
   list_proj.clear();

   // Iterate through each border
   for(const auto& sb : borders)
   {
      bool found = false;

      // Check if the border's zone is already in the list of projections
      for(Projection& p : list_proj)
      {
         if(p.zone == sb->getZone())
         {
            // Update the projection with the border's border and mask
            p.img += sb->drawBorder().clone();
            found = true;
            break;
         }
      }

      // If the zone was not found, add a new projection
      if(!found)
      {
         Projection p;
         p.zone = sb->getZone();
         p.img = sb->drawBorder().clone();
         list_proj.push_back(p);
      }
   }
}

//publish the border to project them
void StaticBorderManager::publishBorder()
{
   // updateBorderStatus();
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
