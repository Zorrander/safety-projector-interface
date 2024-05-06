/*
In this code, we transform RGB coordinates to the depthmap. It uses the Azure Kinect API and uses the video sample acquired during calibration to extract intrinsic camera parameters of the kinect.
First we must find where is the RGB point in the depth image -> getDepthFromRGB()
Once we have the point in the depth image (x,y,depth_value), we get the 3D point by generating a single point pointcloud and transforming it to robot base -> generatePointCloudPOI()
Then, we project this 3D point aligned with the robot frame to the depthmap with the depthmap parameters -> genPointDepthMap
*/

#include "tuni_whitegoods_perception/depth_interface.h"

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <fstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
#include <k4a/k4atypes.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <unity_msgs/InterfacePOI.h>
#include <unity_msgs/poiPCL.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <ros/header.h>
#include <unity_msgs/AffineDepthMap.h>

using namespace std;


 DepthInterface::DepthInterface(ros::NodeHandle* nh, string name_drgb, string name_d, string cam_name):
    nh_(nh),
    it_(*nh),
    camera_name(cam_name),
    depth_rgb_sub_( it_, name_drgb, 1),
    depth_sub_( it_, name_d, 1),
    sync( MySyncPolicy(10), depth_rgb_sub_, depth_sub_)
    {
      ROS_INFO("Fetching parameters for DepthInterface.");

      try {
          if (!nh_->getParam("/robot_base", tf_robot_frame)) {
              ROS_ERROR("Failed to get parameter 'robot_base'");
          }

          if (!nh_->getParam("/depth_camera", tf_depth_frame)) {
              ROS_ERROR("Failed to get parameter 'depth_camera'");
          }

          if (!nh_->getParam("/calibration_folder", calibration_folder)) {
              ROS_ERROR("Failed to get parameter 'calibration_folder'");
          }

          if (!nh_->getParam("/file_recorded", file_recorded)) {
              ROS_ERROR("Failed to get parameter 'file_recorded'");
          }

          if (!nh_->getParam("/master_to_base", master_to_base)) {
              ROS_ERROR("Failed to get parameter 'master_to_base'");
          }

          if (!nh_->getParam("/topic_pcl_master", name_topic_pcl_tf)) {
              ROS_ERROR("Failed to get parameter 'topic_pcl_master'");
          }

          if (!nh_->getParam("/sub_points", name_sub)) {
              ROS_ERROR("Failed to get parameter 'sub_points'");
          }

          if (!nh_->getParam("/number_cam", num_cam)) {
              ROS_ERROR("Failed to get parameter 'number_cam'");
          }
      } catch (ros::InvalidNameException& e) {
        ROS_ERROR("Invalid parameter name: %s", e.what());

      } catch (ros::InvalidParameterException& e) {
          ROS_ERROR("Invalid parameter: %s", e.what());
      }

      ROS_INFO("DephtInterface successfully retrieved parameters. %s", ("/"+camera_name+name_sub).c_str());

      //sync.registerCallback( boost::bind( &DepthInterface::callbackRGBDepth, this, _1, _2) );
      //sub_hand_poi = nh_->subscribe("/hand_tracking/rgb/coordinates", 1, &DepthInterface::handTrackerCallback,this);
      //sub_poi = nh_->subscribe("/interface_poi/buttons", 1, &DepthInterface::poiCallback,this);
      //pub_poi = nh_->advertise<unity_msgs::InterfacePOI> ("/depth_interface/poi_depthmap", 1);
      //pub_poi_pcl = nh_->advertise<sensor_msgs::PointCloud2> ("/depth_interface/poi_pcl", 1);
      //pub_hand_tracker_dm = nh_->advertise<unity_msgs::poiPCL> ("/hand_tracking/dm/coordinates", 1);
      robot_space = Eigen::Affine3d::Identity();
      tf_in = false;
      
      readParamsFile();
      //get ICP params so it can align the pcl from the second camera. For HRC scenario
      //if(num_cam > 1){
      //  readICPFromFile();
      //}

      initTransformToBaseTF();

      pub = nh_->advertise<sensor_msgs::PointCloud2> ("/"+camera_name+name_topic_pcl_tf, 1, true);

      sub_cloud = nh_->subscribe("/"+camera_name+name_sub, 1, &DepthInterface::alignToRobotBase,this); 

      handle = k4a::playback::open(file_recorded.c_str());
      k4aCalibration = handle.get_calibration();
      k4aTransformation = k4a::transformation(k4aCalibration);
      depth_images = false;
      list_points.poi.resize(0);
    }

    void DepthInterface::initTransformToBaseTF()
    {
        try
        {
            // Create a buffer and listener for the TF tree
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener tf_listener(tf_buffer);

            // Wait for the static transform from rgb_camera_link to base
            geometry_msgs::TransformStamped transformStamped;

            transformStamped = tf_buffer.lookupTransform(tf_robot_frame, tf_depth_frame, ros::Time(0), ros::Duration(5.0));

            // Convert the transform to Eigen format
            rgb_to_base = transformStamped.transform;
            ROS_INFO_STREAM(rgb_to_base);
            Eigen::Isometry3d robot_space = tf2::transformToEigen(transformStamped.transform);
            
            tf_in = true;
        }
        catch (tf2::TransformException& ex)
        {
            ROS_ERROR("Failed to obtain transform from rgb_camera_link to base: %s", ex.what());
            tf_in = false;
        }
    }

    void DepthInterface::initTransformToBase()
    {
      rgb_to_base.translation.x = master_to_base[0];
      rgb_to_base.translation.y = master_to_base[1];
      rgb_to_base.translation.z = master_to_base[2];
      rgb_to_base.rotation.x = master_to_base[3];
      rgb_to_base.rotation.y = master_to_base[4];
      rgb_to_base.rotation.z = master_to_base[5];
      rgb_to_base.rotation.w = master_to_base[6];
      ROS_INFO_STREAM(rgb_to_base);
      robot_space = tf2::transformToEigen(rgb_to_base);

      tf_in = true;
    }

    // Align PointCloud to robot base and optionally apply ICP translation for multiple cameras
    void DepthInterface::alignToRobotBase(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        // Create containers for point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        sensor_msgs::PointCloud2 cloud_tf;

        // Convert ROS message to PCL point cloud
        pcl::fromROSMsg(*cloud_msg, *input_cloud);

        // Transform point cloud to robot base frame
        pcl::transformPointCloud(*input_cloud, *transformed_cloud, robot_space);

        // Optionally apply ICP translation for multiple cameras
        if (num_cam > 1) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*transformed_cloud, *icp_cloud, icp_matrix);
            pcl::toROSMsg(*icp_cloud, cloud_tf);
        } else {
            pcl::toROSMsg(*transformed_cloud, cloud_tf);
            cloud_tf.header.stamp = cloud_msg->header.stamp;
        }

        // Set header information for the transformed point cloud
        cloud_tf.header = cloud_msg->header;
        cloud_tf.header.frame_id = tf_robot_frame;

        // Publish the transformed point cloud
        pub.publish(cloud_tf);
    }

    //get the rgb_to_depth and depth images
    void DepthInterface::callbackRGBDepth(const sensor_msgs::ImageConstPtr& rgbdepth_msg, const sensor_msgs::ImageConstPtr& depth_msg)
    {
      cv_bridge::CvImagePtr cv_bridge_depth_rgb = cv_bridge::toCvCopy(rgbdepth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      cv_bridge::CvImagePtr cv_bridge_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      cv_depth_rgb = cv_bridge_depth_rgb->image.clone();
      cv::Mat cv_depth_uint16 = cv_bridge_depth->image.clone();
      //cout<<"depth rgb :"<<cv_depth_rgb.size<<"\n";
      //cout<<"depth :"<<cv_depth.size<<"\n";
      depth_images = true;

    }

    //get rgb points from interface and find them in the global depth map
    /*void DepthInterface::poiCallback(const unity_msgs::InterfacePOIConstPtr& msg)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      list_points.poi.resize(0);
      if(!tf_in)
      {
        initTransformToBase();
      }
      else
      {
        for(int i = 0; i < msg->poi.size(); i++)
        {
          unity_msgs::ElementUI current_elem;
          pcl::PointXYZ p;
          p.x = static_cast<float>(msg->poi[i].elem.x);
          p.y = static_cast<float>(msg->poi[i].elem.y);
          p.z = static_cast<float>(msg->poi[i].elem.z);
          current_elem.id = msg->poi[i].id;
          if(depth_images)
          {
            pcl::PointXYZ pixel_res = getDepthFromRGB(cv_depth_rgb,cv_depth,p);
            pcl::PointXYZ pt_depth = generatePointCloudPOI(cv_depth, pixel_res);
            final_cloud->push_back(pt_depth);
            pcl::PointXYZ final_pt = genPointDepthMap(pt_depth);
            current_elem.elem.x = static_cast<double>(final_pt.x);
            current_elem.elem.y = static_cast<double>(final_pt.y);
            current_elem.elem.z = static_cast<double>(final_pt.z);
            list_points.poi.push_back(current_elem);
          }
        }
        list_points.header = msg->header;
        pub_poi.publish(list_points);
      }
      //publish the points, only for demo so it can be removed later
      sensor_msgs::PointCloud2 cloud_publish;
      pcl::toROSMsg(*final_cloud,cloud_publish);
      cloud_publish.header = msg->header;
      cloud_publish.header.frame_id = tf_robot_frame; //changed here
      pub_poi_pcl.publish(cloud_publish);
    }*/

    //get depthmap parameters
    void DepthInterface::readParamsFile()
    {
      string line;
      string name_file = calibration_folder + "params.txt";
      ifstream calibfile(name_file);
      if(calibfile.is_open())
      {
        getline(calibfile,line);
        ax = stod(line); 
        getline(calibfile,line);
        bx = stod(line);
        getline(calibfile,line);
        ay = stod(line);
        getline(calibfile,line);
        by = stod(line);
        getline(calibfile,line);
        az = stod(line);
        getline(calibfile,line);
        bz = stod(line);
        calibfile.close();
      }
    }


    void DepthInterface::readICPFromFile()
    {
      std::string line;
      std::string calibration_folder;
      ros::param::get("calibration_folder", calibration_folder);
      std::string name_file = calibration_folder + "../ICP/icp.txt";
      std::ifstream icpfile (name_file);
      if(icpfile.is_open())
      {
        for(int i = 0; i < 4; i++)
        {
          for(int j = 0; j < 4; j++)
          {
            getline(icpfile,line);
            double tmp = std::stod(line);
            icp_matrix(i,j) = tmp;
          }
        }
        icpfile.close();
      }
    }

