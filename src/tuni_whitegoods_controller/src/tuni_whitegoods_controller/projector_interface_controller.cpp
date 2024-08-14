/*
- receives perception input (hand) and update model
- track interactions
- update view (projector, camera and robot)
*/

#include "tuni_whitegoods_controller/projector_interface_controller.h"

#include "tuni_whitegoods_msgs/TransformRobotCameraCoordinates.h"
#include "tuni_whitegoods_msgs/Transform3DToPixel.h"
#include "tuni_whitegoods_msgs/TransformPixelTo3D.h"
#include "tuni_whitegoods_msgs/TransformPixelToProjection.h"

#include <integration/VirtualButtonEventArray.h>


    ProjectorInterfaceController::ProjectorInterfaceController(ros::NodeHandle* nh)
        : nh_(nh)
    {	

		// Subscribe to hand detections
        hand_pose_sub = nh_->subscribe("/hand_tracking/rgb/coordinates", 1, &ProjectorInterfaceController::handTrackerCallback, this);
        // Subscribe to moving table detections
        moving_table_pose_sub = nh_->subscribe("/odin/projector_interface/moving_table", 1, &ProjectorInterfaceController::movingTableTrackerCallback, this);
        // Subscribe to object detections

        // Subscribe to commands coming from OpenFlow or custom scheduler
        service_borders = nh->advertiseService("/execution/projector_interface/integration/services/list_static_border_status", &ProjectorInterfaceController::getBordersService, this); 
        pub_button_event = nh->advertise<integration::VirtualButtonEventArray> ("/execution/projector_interface/integration/topics/virtual_button_event_array", 1);

        // Start clients for transform       
        client_world_coordinates = nh_->serviceClient<tuni_whitegoods_msgs::TransformRobotCameraCoordinates>("transform_world_coordinates_frame");
	    client_3D_to_pixel = nh_->serviceClient<tuni_whitegoods_msgs::Transform3DToPixel>("transform_3D_to_pixel");
	    client_pixel_to_3D = nh_->serviceClient<tuni_whitegoods_msgs::TransformPixelTo3D>("transform_pixel_to_3D");
	    client_projector_point = nh_->serviceClient<tuni_whitegoods_msgs::TransformPixelToProjection>("transform_point_to_project");
	    client_reverse_projector_point = nh_->serviceClient<tuni_whitegoods_msgs::TransformPixelToProjection>("reverse_transform_point_to_project");

	    // Initialize model 
	    model_ = std::make_unique<ProjectorInterfaceModel>();
	    model_->add_zone("table");

	    this->model_->attach(shared_from_this());

	    // Initialize views
	    views.push_back(std::make_unique<Projector>());
	    views.push_back(std::make_unique<CameraView>());  
	    views.push_back(std::make_unique<RobotView>()); 
 
		std::for_each(views.begin(), views.end(), [](auto& view) {
		    view->init();
		});

	    ROS_INFO("StaticBorderManager running");

    }


    ProjectorInterfaceController::~ProjectorInterfaceController() {
        if (model_) {
            model_->detach(); 
        }
    }

	void ProjectorInterfaceController::notify(){
		std::for_each(views.begin(), views.end(), [](auto& view) {
		    view->update();
		});
	}

	void ProjectorInterfaceController::createBorderLayout(int rows, int cols, float sf_factor, bool adjacent, std_msgs::ColorRGBA status_booked, std_msgs::ColorRGBA status_free, std_msgs::ColorRGBA status_operator){
		ROS_INFO("createBorderLayout");
	}

	void ProjectorInterfaceController::movingTableTrackerCallback(const tuni_whitegoods_msgs::HandsState& msg)
    {
        ROS_INFO("movingTableTrackerCallback");
    }
	
    void ProjectorInterfaceController::handTrackerCallback(const tuni_whitegoods_msgs::HandsState& msg)
    {
    	ROS_INFO("handTrackerCallback");
    	/*
        cv::Mat cv_viz_depth_hands = cv_viz_depth.clone();
        for(int i = 0; i < msg->name.size(); i++)
        {

          tuni_whitegoods_msgs::TransformPixelTo3D srv_pixel_to_3D;
          srv_pixel_to_3D.request.u = msg->pts[i].x;
          srv_pixel_to_3D.request.v = msg->pts[i].y;


          if (client_pixel_to_3D.call(srv_pixel_to_3D))
          {
              // Transform to robot coordinates frame

              geometry_msgs::PoseStamped in_point_stamped;
              in_point_stamped.header.frame_id = "rgb_camera_link";
              in_point_stamped.header.stamp = ros::Time(0);
              in_point_stamped.pose.position.x = srv_pixel_to_3D.response.x;
              in_point_stamped.pose.position.y = srv_pixel_to_3D.response.y;
              in_point_stamped.pose.position.z = srv_pixel_to_3D.response.z; 

              tuni_whitegoods_msgs::TransformRobotCameraCoordinates srv;
              srv.request.in_point_stamped = in_point_stamped;
              srv.request.target_frame = "base";
              if (client.call(srv))
              {
                // Create Rviz marker 
                visualization_msgs::Marker marker;
                marker.header.frame_id = "base";
                marker.header.stamp = ros::Time();
                marker.id = 1;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose = srv.response.out_point_stamped.pose;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                vis_pub.publish( marker );
              }
              else
              {
                ROS_ERROR("Failed to call service transform_pixel_to_3D");
              }

          }
          else
          {
            ROS_ERROR("Failed to call service transform_world_coordinates_frame");
          }
        
        }

			border_crossed = false;
		   button_pressed = false;
		   list_hand_violation.clear();
		   std::string handType = msg->header.frame_id;
		   for (auto& border : borders){
		      const cv::Point border_center = border->getCenter();

		      for (const auto& hand_point : msg->pts)
		      {
		          geometry_msgs::PoseStamped hand_pose;
		          hand_pose.pose.position.x = hand_point.x;
		          hand_pose.pose.position.y = hand_point.y;

		         if (handType == "Left") {
		            cv::Point left_hand_position(static_cast<int>(hand_point.x), static_cast<int>(hand_point.y));

		            float distance = cv::norm(left_hand_position - border_center);

		            if (distance < border->getBorderDiagonal()*0.75)
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

		            float distance = cv::norm(right_hand_position - border_center);

		            if (distance < border->getBorderDiagonal()*0.75)
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
		        border->changeThickness(-1);
		        
		         //if (border->occupied){
		         //   border->changeBorderColor(stat_operator);
		         //}
		         //else if (border->booked){
		          //  border->changeThickness(-1);
		            //Communicate with openflow
		            //pub_border_violation.publish(msg_border);
		            //pub_border_polygon.publish(bord);
		            //pub_pose_violation.publish(pose_location);
		         //} else {
		         //   border->changeThickness(2);
		         //} 

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

        */
    }



	void ProjectorInterfaceController::addBorder(std::string r_id, std::string z, int pos_row, int pos_col, geometry_msgs::PolygonStamped bord, std::string b_topic, std_msgs::ColorRGBA b_color, bool filling, int thic, ros::Duration life, bool track) {
	   

	    std::cout<<"StaticBorderManager::addBorder\n";

	    /*
	     geometry_msgs::Point topLeftCornerPt;
	     topLeftCornerPt.x = bord.polygon.points[0].x; 
	     topLeftCornerPt.y = bord.polygon.points[0].y;

	     geometry_msgs::Point bottomRightCornerPt;
	     bottomRightCornerPt.x = bord.polygon.points[1].x; 
	     bottomRightCornerPt.y = bord.polygon.points[1].y;

	    // Also compute camera coordinates for detections

	    // TOP LEFT
	    // Convert to 3D camera coordinates frame
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
	  
	   // BOTTOM RIGHT
	   in_point_stamped.header.frame_id = "base";
	   in_point_stamped.header.stamp = ros::Time(0);
	   in_point_stamped.pose.position.x = bottomRightCornerPt.x;
	   in_point_stamped.pose.position.y = bottomRightCornerPt.y;

	   srv.request.in_point_stamped = in_point_stamped;
	   srv.request.target_frame = "rgb_camera_link";
	   client.call(srv);
	 
	   srv_3D_to_pixel.request.x = srv.response.out_point_stamped.pose.position.x;
	   srv_3D_to_pixel.request.y = srv.response.out_point_stamped.pose.position.y;
	   srv_3D_to_pixel.request.z = 1.311792;
	   client_3D_to_pixel.call(srv_3D_to_pixel);

	   sb->bottom_right_cam_point.x = srv_3D_to_pixel.response.u;
	   sb->bottom_right_cam_point.y = srv_3D_to_pixel.response.v;


	   // Add border to the list of borders
	   //borders.push_back(sb);
	   
	  // ///////////  model->addBorder()

	  */
	}

	//Book a robot border by its id
	void ProjectorInterfaceController::bookBorderRobot(std::string id)
	{
	   ROS_INFO("BOOKING BORDER %s", id.c_str());

	   /*
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

	   */
	}

	//Book a border for the operator. It signals the operator that an object can be picked by using a different color.
	void ProjectorInterfaceController::bookBorderOperator(std::string id)
	{
		 ROS_INFO("bookBorderOperator");
		/*
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

	   */
	}

	//release a border booked by the robot
	void ProjectorInterfaceController::releaseRobotBorder(std::string id, int status)
	{

	   //release booking and change color
	   ROS_INFO("RELEASING BORDER %s -> %i", id.c_str(), status);
	   /*
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
	   */
	}

	//release a booking made by the operator
	void ProjectorInterfaceController::releaseOperatorBorder(std::string id, int status)
	{
		 ROS_INFO("releaseOperatorBorder");

		 /*
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

	   */
	}

	/**
	 * Get the IDs of adjacent borders based on row and column indices.
	 * 
	 * @param row The row index.
	 * @param col The column index.
	 * @return A vector of IDs of adjacent borders.
	 */
	std::vector<std::string> ProjectorInterfaceController::getAdjacentBorders(int row, int col)
	{
		ROS_INFO("getAdjacentBorders");

		
	   std::vector<std::string> list_adj;
	   /*
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
	   */
	   return list_adj;
	}






	bool ProjectorInterfaceController::getBordersService(integration::ListStaticBordersStatus::Request& req, integration::ListStaticBordersStatus::Response& res)
	{
	   ROS_INFO("Checking border status...");

	   /*
	   res.status_borders.clear();

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
	   */
	   return true;
	}



 //   void ProjectorInterfaceController::callback_button(const unity_msgs::ElementUI::ConstPtr &msg) {
 //       cout << "msg.zone " << msg->zone << endl;
        /*
        if (msg->zone.empty()) {
            process_button(msg->center.position.x, msg->center.position.y, *msg);
        } else if (msg->zone == "moving_table") {
            double world_center_x = (moving_table.tl.x + moving_table.bl.x) * msg->center.position.x;
            double world_center_y = (moving_table.tl.y + moving_table.tr.y) * msg->center.position.y;
            process_button(world_center_x, world_center_y, *msg);
        }
        object_detection_pub.publish(bridge_interface.cv2_to_imgmsg(cv_image, "bgr8"));
        */
  //  }

  //  void process_button(double center_x, double center_y, const unity_msgs::ElementUI &msg) {
  //      cout << "process_button " << center_x << ", " << center_y << endl;
        /*
        // Create Rviz marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base";
        marker.header.stamp = ros::Time::now();
        marker.id = msg.id;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = center_x;
        marker.pose.position.y = center_y;
        // Set the dimensions of the cylinder
        marker.scale.x = msg.radius / 500;
        marker.scale.y = msg.radius / 500;
        marker.scale.z = 0.01;  // Height of the cylinder
        // Set the color of the cylinder
        marker.color.r = msg.button_color.r;
        marker.color.g = msg.button_color.g;
        marker.color.b = msg.button_color.b;
        marker.color.a = 1.0;
        vis_pub.publish(marker);

        // Create the input pose stamped message
        geometry_msgs::PoseStamped in_point_stamped;
        in_point_stamped.header.frame_id = "base";
        in_point_stamped.header.stamp = ros::Time(0);
        in_point_stamped.pose.position.x = center_x;
        in_point_stamped.pose.position.y = center_y;

        // Perform the transformation
        tuni_whitegoods_msgs::srv::TransformRobotCameraCoordinates srv;
        srv.request.in_point_stamped = in_point_stamped;
        srv.request.target_frame = "rgb_camera_link";

        if (transform_world_coordinates.call(srv)) {
            auto cam_coordinates = srv.response.out_point_stamped.pose.position;

            // Project to 2D image coordinates
            tuni_whitegoods_msgs::srv::Transform3DToPixel srv_pixel;
            srv_pixel.request.x = cam_coordinates.x;
            srv_pixel.request.y = cam_coordinates.y;
            srv_pixel.request.z = cam_coordinates.z;

            if (project_3D_to_pixel.call(srv_pixel)) {
                int center_cam_point_x = srv_pixel.response.u;
                int center_cam_point_y = srv_pixel.response.v;
                circle(cv_image, Point(center_cam_point_x, center_cam_point_y), int(msg.radius), Scalar(255, 255, 255), 2);

                Button b(msg.id, msg.zone, msg.name, msg.description, msg.text, msg.button_color, msg.text_color,
                         Point(center_cam_point_x, center_cam_point_y), msg.radius, msg.hidden);

                bool add_success = false;
                if (_list_interface.empty()) {
                    vector<Button> tmp;
                    tmp.push_back(b);
                    InterfaceUI ui("default", msg.zone, "default_interface", "interface by default", tmp);
                    _list_interface.push_back(ui);
                    add_success = true;
                } else {
                    for (auto &i : _list_interface) {
                        cout << i.get_zone() << endl;
                        cout << b.get_zone() << endl;
                        if (i.get_zone() == b.get_zone()) {
                            i.add_button(b);
                            add_success = true;
                        }
                    }
                }
                if (!add_success) {
                    vector<Button> tmp;
                    tmp.push_back(b);
                    uuid_t uuid;
                    uuid_generate(uuid);
                    char uuid_str[37];
                    uuid_unparse(uuid, uuid_str);
                    InterfaceUI ui(uuid_str, msg.zone, "default_interface", "interface by default", true, tmp);
                    _list_interface.push_back(ui);
                }
            }
        }
        */
   // }
    
   // void ProjectorInterfaceController::callback_button_color(const unity_msgs::ElementUI::ConstPtr &msg) {
    //	ROS_INFO("callback_button_color");
    	/*
        for (auto &i : _list_interface) {
            if (!i.get_hidden()) {
                i.modify_button_color(*msg);
            }
        }
        */
   // }

