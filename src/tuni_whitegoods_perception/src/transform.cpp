/*
Class transforming the pointcloud coming from the kinect camera
The goal is to align the point cloud to the robot base so the scene is flat*/
#include <ros/ros.h>
// PCL specific includes
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
#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <unity_msgs/InterfacePOI.h>
#include <unity_msgs/ElementUI.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <ros/header.h>
#include <geometry_msgs/TransformStamped.h>


class Transform
{
  private:
      ros::NodeHandle nh_;
      std::string tf_robot_frame;
      std::string tf_depth_frame;
      std::string tf_rgb_frame;
      std::string name_topic_pcl_tf;
      std::string name_topic_tf_poi;
      std::string name_sub;
      std::vector<double>  master_to_base;
      std::string tf_depth_camera;
      int num_cam;
      bool icp;
      ros::Subscriber sub_cloud;
      ros::Subscriber sub_cloud_poi;
      ros::Subscriber sub_poi;
      ros::Subscriber sub_sub;
      typedef image_transport::SubscriberFilter ImageSubscriber;
      ImageSubscriber depth_rgb_sub_;
      ImageSubscriber depth_sub_;
      ros::Publisher pub;
      ros::Publisher pub_d;
      ros::Publisher pub_poi;
      Eigen::Matrix4d icp_matrix;
      Eigen::Affine3d robot_space;
      bool tf_in;
      pcl::console::TicToc tic;
      k4a::playback handle;
      k4a::calibration k4aCalibration;
      k4a::transformation k4aTransformation;
      image_transport::Subscriber image_sub_;
      image_transport::Subscriber image_depth_sub_;
      std::vector<geometry_msgs::Point> list_poi;
      std::vector<pcl::PointXYZ> pix_depth;
      pcl::PointCloud<pcl::PointXYZ> poi_interface;
      geometry_msgs::TransformStamped transformStamped;
      geometry_msgs::TransformStamped transformStamped_rgb;
      geometry_msgs::Transform rgb_to_base;
      Eigen::Matrix4d robot_frame;
      Eigen::Matrix4d camera_depth_frame;
      int compare;
      
      sensor_msgs::PointCloud2 poi_cloud;

  public:
    Transform()
    {



      try {

          if (!nh_.getParam("robot_base", tf_robot_frame)) {
              ROS_ERROR("Failed to get parameter 'robot_base'");
          }

          if (!nh_.getParam("depth_camera", tf_depth_frame)) {
              ROS_ERROR("Failed to get parameter 'depth_camera'");
          }

          if (!nh_.getParam("topic_pcl_master", name_topic_pcl_tf)) {
              ROS_ERROR("Failed to get parameter 'topic_pcl_master'");
          }

          if (!nh_.getParam("master_to_base", master_to_base)) {
              ROS_ERROR("Failed to get parameter 'master_to_base'");
          }

          if (!nh_.getParam("sub_points", name_sub)) {
              ROS_ERROR("Failed to get parameter 'sub_points'");
          }

          if (!nh_.getParam("number_cam", num_cam)) {
              ROS_ERROR("Failed to get parameter 'number_cam'");
          }

      } catch (ros::InvalidNameException& e) {
        ROS_ERROR("Invalid parameter name: %s", e.what());

      } catch (ros::InvalidParameterException& e) {
          ROS_ERROR("Invalid parameter: %s", e.what());

      }

      ROS_INFO("Transform successfully retrieved parameters.");

      //get ICP params so it can align the pcl from the second camera. For HRC scenario
      if(num_cam > 1){
        readICPFromFile();
      }
      
      initTransformToBase();

      pub = nh_.advertise<sensor_msgs::PointCloud2> (name_topic_pcl_tf, 1, true);
      sub_cloud = nh_.subscribe(name_sub, 1, &Transform::alignToRobotBase,this);       
      list_poi.resize(0);
      robot_space = Eigen::Affine3d::Identity();
    }

    // Function to initialize transformation to base using tf2 static transform
    void initTransformToBaseTF() {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);

        // Wait for the static transform to become available
        geometry_msgs::TransformStamped static_transform;
        try {
            static_transform = tfBuffer.lookupTransform(tf_robot_frame, tf_depth_frame, ros::Time(0), ros::Duration(5.0));
            rgb_to_base = static_transform.transform;
            ROS_INFO_STREAM(rgb_to_base);
            Eigen::Isometry3d mat = tf2::transformToEigen(static_transform.transform);
            robot_frame = mat.matrix();
            tf_in = true;
        } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            tf_in = false;
            return;
        }
    }

    void initTransformToBase()
    {
      rgb_to_base.translation.x = master_to_base[0];
      rgb_to_base.translation.y = master_to_base[1];
      rgb_to_base.translation.z = master_to_base[2];
      rgb_to_base.rotation.x = master_to_base[3];
      rgb_to_base.rotation.y = master_to_base[4];
      rgb_to_base.rotation.z = master_to_base[5];
      rgb_to_base.rotation.w = master_to_base[6];
      ROS_INFO_STREAM(rgb_to_base);
      Eigen::Isometry3d mat = tf2::transformToEigen(rgb_to_base);
      robot_frame = mat.matrix();
      tf_in = true;
    }

    // Align PointCloud to robot base and optionally apply ICP translation for multiple cameras
    void alignToRobotBase(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        // Create containers for point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        sensor_msgs::PointCloud2 cloud_tf;

        // Convert ROS message to PCL point cloud
        pcl::fromROSMsg(*cloud_msg, *input_cloud);

        // Transform point cloud to robot base frame
        pcl::transformPointCloud(*input_cloud, *transformed_cloud, robot_frame);

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


    void writeICPAsText(const Eigen::Matrix4d & matrix)
    {
      std::ofstream ofile;
      ofile.open("/home/altair/catkin_abb/src/transform_pcl/config/icp.txt", std::ios::app);
      for(int i = 0; i < 4;i++)
      {
        for(int j = 0; j < 4; j++)
        {
          double tmp = matrix.coeff(i,j);
          ofile << tmp << std::endl;
        }
      }
      ofile.close();
    }

    void readICPFromFile()
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

    //print rotation and translation matrix
    void print4x4Matrix (const Eigen::Matrix4d & matrix)
    {
      printf ("Rotation matrix : \n");
      printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
      printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
      printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
      printf ("Translation vector :\n");
      printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
    }
}; 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_trans");
  ROS_INFO("Starting Transform node to align point cloud with robot base...");
  Transform transform_pcl;
  ROS_INFO("Transform node running.");
  ros::spin();

  return 0;
}