/*
In this code, we transform RGB coordinates to the depthmap. It uses the Azure Kinect API and uses the video sample acquired during calibration to extract intrinsic camera parameters of the kinect.
First we must find where is the RGB point in the depth image -> getDepthFromRGB()
Once we have the point in the depth image (x,y,depth_value), we get the 3D point by generating a single point pointcloud and transforming it to robot base -> generatePointCloudPOI()
Then, we project this 3D point aligned with the robot frame to the depthmap with the depthmap parameters -> genPointDepthMap
*/

#include "tuni_whitegoods_perception/depth_interface.h"
#include "tuni_whitegoods_msgs/TransformPixelTo3D.h"
#include "tuni_whitegoods_msgs/TransformRobotCameraCoordinates.h"

using namespace std;


 DepthInterface::DepthInterface(ros::NodeHandle* nh):
    nh_(nh),
    it_(*nh),
    tfBuffer(),  // Initialize tfBuffer
    tfListener(new tf2_ros::TransformListener(tfBuffer))
    {
      client = nh_->serviceClient<tuni_whitegoods_msgs::TransformRobotCameraCoordinates>("transform_world_coordinates_frame");
      client_pixel_to_3D = nh_->serviceClient<tuni_whitegoods_msgs::TransformPixelTo3D>("transform_pixel_to_3D");

      sub_hand_poi = nh_->subscribe("/hand_tracking/rgb/coordinates", 1, &DepthInterface::handTrackerCallback,this);

      img_callback = nh_->subscribe("/depth_to_rgb/image_raw", 1, &DepthInterface::callbackRGBDepth,this);

      sub_scene = nh_->subscribe("odin/visualization/scene_detection", 1, &DepthInterface::depthSceneCallback, this);

      pub_full_scene = it_.advertise("odin/visualization/full_scene_detection", 1);
      pub_hand_tracker_dm = nh_->advertise<unity_msgs::poiPCL> ("/hand_tracking/dm/coordinates", 1);

      vis_pub = nh->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

      list_points.poi.resize(0);
    }


    void DepthInterface::depthSceneCallback(const sensor_msgs::ImageConstPtr& depth_msg){
        cv_bridge::CvImagePtr cv_viz_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::BGR8);
        cv_viz_depth = cv_viz_depth_ptr->image.clone();
    }

    //get the rgb_to_depth and depth images
    void DepthInterface::callbackRGBDepth(const sensor_msgs::ImageConstPtr& rgbdepth_msg)
    {
      cv_bridge::CvImagePtr cv_bridge_depth_rgb = cv_bridge::toCvCopy(rgbdepth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      cv_depth_rgb = cv_bridge_depth_rgb->image.clone();
    }


    void DepthInterface::handTrackerCallback(const unity_msgs::poiPCLConstPtr& msg)
    {
      list_points_hand.pts.clear();

        cv::Mat cv_viz_depth_hands = cv_viz_depth.clone();
        for(int i = 0; i < msg->pts.size(); i++)
        {
          geometry_msgs::Point current_elem;
          current_elem.x = msg->pts[i].x;
          current_elem.y = msg->pts[i].y;

          list_points_hand.pts.push_back(current_elem);
          cv::circle(cv_viz_depth_hands, cv::Point(msg->pts[i].x, msg->pts[i].y), 10, cv::Scalar(255, 0, 0), 2);
          sensor_msgs::ImagePtr viz_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_viz_depth_hands).toImageMsg();
          pub_full_scene.publish(viz_msg);

          // Get the depth value at the RGB point
          // uint16_t depth_value = cv_depth_rgb.at<uint16_t>(msg->pts[i].x, msg->pts[i].y);

          // Assuming depth image is in millimeters, convert to meters
          // float depth_in_meters = depth_value / 1000.0;

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

        list_points_hand.header = msg->header;
        //publish hand coordinates in the depthmap
        pub_hand_tracker_dm.publish(list_points_hand);

    }


