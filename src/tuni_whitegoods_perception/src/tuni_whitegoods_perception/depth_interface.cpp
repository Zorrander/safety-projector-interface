/*
In this code, we transform RGB coordinates to the depthmap. It uses the Azure Kinect API and uses the video sample acquired during calibration to extract intrinsic camera parameters of the kinect.
First we must find where is the RGB point in the depth image -> getDepthFromRGB()
Once we have the point in the depth image (x,y,depth_value), we get the 3D point by generating a single point pointcloud and transforming it to robot base -> generatePointCloudPOI()
Then, we project this 3D point aligned with the robot frame to the depthmap with the depthmap parameters -> genPointDepthMap
*/

#include "tuni_whitegoods_perception/depth_interface.h"

using namespace std;


 DepthInterface::DepthInterface(ros::NodeHandle* nh, string name_drgb, string name_d):
    nh_(nh),
    it_(*nh),
    depth_rgb_sub_( it_, name_drgb, 1),
    depth_sub_( it_, name_d, 1),
    sync( MySyncPolicy(10), depth_rgb_sub_, depth_sub_),
    tfBuffer(),  // Initialize tfBuffer
    tfListener(new tf2_ros::TransformListener(tfBuffer))
    {

      sync.registerCallback( boost::bind( &DepthInterface::callbackRGBDepth, this, _1, _2) );
      sub_hand_poi = nh_->subscribe("/hand_tracking/rgb/coordinates", 1, &DepthInterface::handTrackerCallback,this);

      sub_scene = nh_->subscribe("odin/visualization/scene_detection", 1, &DepthInterface::depthSceneCallback, this);
      pub_full_scene = it_.advertise("odin/visualization/full_scene_detection", 1);

      pub_hand_tracker_dm = nh_->advertise<unity_msgs::poiPCL> ("/hand_tracking/dm/coordinates", 1);

      list_points.poi.resize(0);
    }


    void DepthInterface::depthSceneCallback(const sensor_msgs::ImageConstPtr& depth_msg){
        cv_bridge::CvImagePtr cv_viz_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::BGR8);
        cv_viz_depth = cv_viz_depth_ptr->image.clone();
    }

    //get the rgb_to_depth and depth images
    void DepthInterface::callbackRGBDepth(const sensor_msgs::ImageConstPtr& rgbdepth_msg, const sensor_msgs::ImageConstPtr& depth_msg)
    {
      cv_bridge::CvImagePtr cv_bridge_depth_rgb = cv_bridge::toCvCopy(rgbdepth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      cv_bridge::CvImagePtr cv_bridge_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      cv_depth_rgb = cv_bridge_depth_rgb->image.clone();
      cv_depth = cv_bridge_depth->image.clone();
    }


    void DepthInterface::handTrackerCallback(const unity_msgs::poiPCLConstPtr& msg)
    {
      list_points_hand.pts.clear();

        cv::Mat cv_viz_depth_hands = cv_viz_depth.clone();
        for(int i = 0; i < msg->pts.size(); i++)
        {

          cv::circle(cv_viz_depth_hands, cv::Point(msg->pts[i].x, msg->pts[i].y), 10, cv::Scalar(255, 0, 0), 2);

          geometry_msgs::Point current_elem;

          // Get the depth value at the RGB point
          uint16_t depth_value = cv_depth_rgb.at<uint16_t>(msg->pts[i].x, msg->pts[i].y);

          // Assuming depth image is in millimeters, convert to meters
          float depth_in_meters = depth_value / 1000.0;

          float x = (msg->pts[i].x - 640.501220703125) * 1.311792 / 607.4446411132812;
          float y = (msg->pts[i].y - 362.7770080566406) * 1.311792 / 607.5540771484375;
          float z =  1.311792;
          
          // Transform to robot coordinates frame

           geometry_msgs::PoseStamped in_point_stamped, out_point_stamped;

           in_point_stamped.header.frame_id = "rgb_camera_link";
           in_point_stamped.header.stamp = ros::Time(0);
           in_point_stamped.pose.position.x = x;
           in_point_stamped.pose.position.y = y;
           in_point_stamped.pose.position.z = z; 

           try {
                 out_point_stamped = tfBuffer.transform(in_point_stamped, "base");
                 current_elem.x = out_point_stamped.pose.position.x;
                 current_elem.y = out_point_stamped.pose.position.y;
                 current_elem.z = out_point_stamped.pose.position.z;
                 ROS_INFO("3D Point in robot coordinate system: X = %.3f, Y = %.3f, Z = %.3f", current_elem.x , current_elem.y, current_elem.z);
                list_points_hand.pts.push_back(current_elem);
              } catch (tf2::TransformException &ex) {
                 ROS_WARN("TF2 Transform Exception: %s", ex.what());    
            }         
        }

        list_points_hand.header = msg->header;
        //publish hand coordinates in the depthmap
        pub_hand_tracker_dm.publish(list_points_hand);

        sensor_msgs::ImagePtr viz_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_viz_depth_hands).toImageMsg();
        pub_full_scene.publish(viz_msg);

    }


