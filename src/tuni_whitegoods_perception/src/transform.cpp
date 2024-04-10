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
      std::string name_tf_robot;
      int num_cam;
      bool icp;
      ros::Subscriber sub_cloud;
      ros::Subscriber sub_cloud_poi;
      ros::Subscriber sub_poi;
      ros::Subscriber sub_sub;
      typedef image_transport::SubscriberFilter ImageSubscriber;
      ImageSubscriber depth_rgb_sub_;
      ImageSubscriber depth_sub_;
      std_msgs::Header sub_header;
      ros::Publisher pub;
      ros::Publisher pub_d;
      ros::Publisher pub_poi;
      Eigen::Matrix4d icp_matrix;
      Eigen::Affine3d robot_space;
      bool tf_in;
      bool tf_rgb_sub;
      bool tf_rgb_master;
      bool tf_rgb;
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
      //params from ROS
      ros::param::get("robot_base", tf_robot_frame);
      ros::param::get("topic_pcl_master", name_topic_pcl_tf);
      ros::param::get("sub_points", name_sub);
      ros::param::get("name_transform_robot_space", name_tf_robot);
      ros::param::get("number_cam", num_cam);

      //get ICP params so it can align the pcl from the second camera. For HRC scenario
      readICPFromFile();
      initTransformToBase();
      tf_rgb_sub = false;
      tf_rgb_master = false;
      tf_rgb = false;
      //To subscribe to 2nd camera if there is one
      if(num_cam > 1)
      {
        sub_sub = nh_.subscribe("/voxel_grid_master/output", 1, &Transform::subPclCallback,this);
      }
      pub = nh_.advertise<sensor_msgs::PointCloud2> (name_topic_pcl_tf, 1, true);
      //sub_poi = nh_.subscribe("/interface_poi", 1, &Transform::poiCb,this);
      sub_cloud = nh_.subscribe(name_sub, 1, &Transform::alignToRobotBase,this);       
      list_poi.resize(0);
      robot_space = Eigen::Affine3d::Identity();
    }
    ~Transform()
    {
    }
    //get sub pcl header to synchronize timestamps on the publishers
    void subPclCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        sub_header = cloud_msg->header;
    }
    //read camera robot calibration. This matrix is the base for aligning the pcl camera to the robot frame 
    void initTransformToBase()
    {
      std::vector<double> tf_list;
      nh_.getParam(name_tf_robot, tf_list);
      rgb_to_base.translation.x = tf_list[0];
      rgb_to_base.translation.y = tf_list[1];
      rgb_to_base.translation.z = tf_list[2];
      rgb_to_base.rotation.x = tf_list[3];
      rgb_to_base.rotation.y = tf_list[4];
      rgb_to_base.rotation.z = tf_list[5];
      rgb_to_base.rotation.w = tf_list[6];
      Eigen::Isometry3d mat = tf2::transformToEigen(rgb_to_base);
      robot_frame = mat.matrix();
      tf_in = true;
    }


    //align PointCloud camera_rgb -> depth_frame (for HRC) then to robot base and depending on the camera, use icp translation to align both pointcloud.
    void alignToRobotBase(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
      // Create a container for the data.
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr depth_tf_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      sensor_msgs::PointCloud2 cloud_tf;
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
      pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
      tic.tic();
      pcl::transformPointCloud(*temp_cloud,*final_cloud,robot_frame);
      if(num_cam > 1)
      {
        pcl::transformPointCloud(*final_cloud, *icp_cloud, icp_matrix);
        pcl::toROSMsg(*icp_cloud,cloud_tf);
      } 
      else
      {
        pcl::toROSMsg(*final_cloud,cloud_tf);
        cloud_tf.header.stamp = sub_header.stamp;
      }
      cloud_tf.header = cloud_msg->header;
      cloud_tf.header.frame_id = tf_robot_frame;
      pub.publish(cloud_tf);
      
    }

    //icp matrix hard coded - deprecated
    void initMatrixTransform()
    {
      icp_matrix = Eigen::Matrix4d::Identity();
      icp_matrix (0, 0) = 0.995;
      icp_matrix (0, 1) = -0.019;
      icp_matrix (0, 2) = 0.096;
      icp_matrix (1, 0) = 0.002;
      icp_matrix (1, 1) = 0.986;
      icp_matrix (1, 2) = 0.167;
      icp_matrix (2, 0) = -0.097;
      icp_matrix (2, 1) = -0.166;
      icp_matrix (2, 2) = 0.981;
      icp_matrix (0, 3) = -0.340;
      icp_matrix (1, 3) = -0.493;
      icp_matrix (2, 3) = 0.293;
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
  Transform transform_pcl;
  ros::spin();

  return 0;
}