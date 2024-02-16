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
//#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
#include <ros/header.h>


class Transform
{
  private:
      ros::NodeHandle nh_;
      //image_transport::ImageTransport it_;
      std::string tf_robot_frame;
      std::string tf_depth_frame;
      std::string tf_rgb_frame;
      std::string name_topic_pcl_tf;
      std::string name_topic_tf_poi;
      std::string name_sub;
      std::string tf_cam;
      int num_cam;
      bool icp;
      ros::Subscriber sub_cloud;
      ros::Subscriber sub_cloud_poi;
      ros::Subscriber sub_poi;
      ros::Subscriber sub_sub;
      typedef image_transport::SubscriberFilter ImageSubscriber;
      ImageSubscriber depth_rgb_sub_;
      ImageSubscriber depth_sub_;
      tf2_ros::Buffer tfBuffer;
      //tf2_ros::Buffer tfBuffer_rgb;
      //tf2_ros::TransformListener tfListener;
      //tf2_ros::TransformListener tfListener_rgb;
      std::shared_ptr<tf2_ros::TransformListener> tfListener;
      //tf2_ros::Buffer tf_buffer_;
      std_msgs::Header sub_header;
      //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
      //message_filters::Synchronizer< MySyncPolicy > sync;
      ros::Publisher pub;
      ros::Publisher pub_d;
      ros::Publisher pub_poi;
      //tf::TransformListener listener;
      //tf::StampedTransform transform;
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
      Eigen::Matrix4d robot_frame;
      Eigen::Matrix4d camera_depth_frame;
      int compare;
      
      sensor_msgs::PointCloud2 poi_cloud;

  public:
    Transform()
    {
      tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);
      ros::param::get("~tf_cam", tf_cam);
      ros::param::get("robot_base", tf_robot_frame);
      ros::param::get("~depth_camera", tf_depth_frame);
      ros::param::get("~topic_pcl_master", name_topic_pcl_tf);
      ros::param::get("~sub_points", name_sub);
      ros::param::get("~icp", icp);
      //std::cout<<icp<<"\n";
      readICPFromFile();
      tf_in = false;
      tf_rgb_sub = false;
      tf_rgb_master = false;
      tf_rgb = false;
      if(icp)
      {
        sub_sub = nh_.subscribe("/voxel_grid_master/output", 1, &Transform::subPclCallback,this);
      }
      // compare = tf_robot_frame.compare("base_robot_master");
      // if(compare == 0)
      // {
      //   name_topic_tf = "/kinect_master/cloud_robot_frame";
      //   pub_poi = nh_.advertise<sensor_msgs::PointCloud2> ("/kinect_master/poi_robot_frame", 1);
      //   //sub_sub = nh_.subscribe("/voxel_grid_sub/output", 1, &Transform::subPclCallback,this);
      // }
      // else
      // {
      //   name_topic_tf = "/kinect_sub/cloud_robot_frame";
      //   //sub_sub = nh_.subscribe("/voxel_grid_master/output", 1, &Transform::subPclCallback,this);
      // }
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


    //align PointCloud camera_rgb -> depth_frame then to robot base and depending on the camera, use icp translation to align both pointcloud.
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
      if(!tf_in)
      {
        listenTransform();
      }
      else
      {
        pcl::transformPointCloud(*temp_cloud,*final_cloud,robot_frame);
        if(icp != 0)
        {
          //pcl::transformPointCloud(*temp_cloud,*depth_tf_cloud,camera_frame_master);
          //std::cout<<"icp: "<<tic.toc()<<"ms\n";
          pcl::transformPointCloud(*final_cloud, *icp_cloud, icp_matrix);
          pcl::toROSMsg(*icp_cloud,cloud_tf);
          // cloud_tf.header = cloud_msg->header;
          // cloud_tf.header.frame_id = tf_depth_frame;
          //std::cout<<"transform sub: "<<cloud_tf.header<<"\n";
          //std::cout<<"transform sub: "<<cloud_msg->header<<"ms\n";
          //std::cout<<"sub: "<<tic.toc()<<"ms\n";
        } 
        else
        {
          pcl::toROSMsg(*final_cloud,cloud_tf);
          //cloud_tf.header = cloud_msg->header;
          cloud_tf.header.stamp = sub_header.stamp;
          //cloud_tf.header.frame_id = tf_depth_frame;
          //std::cout<<"transform master: "<<cloud_tf.header<<"\n";
          //std::cout<<"transform master : "<<cloud_msg->header<<"ms\n";
          //std::cout<<"master: "<<tic.toc()<<"ms\n";
        }
        cloud_tf.header = cloud_msg->header;
        cloud_tf.header.frame_id = tf_robot_frame;
        pub.publish(cloud_tf);
      }
    }

    void listenTransform()
    {
      bool error = false;
      Eigen::Affine3d cmp = Eigen::Affine3d::Identity();
      while(cmp.isApprox(robot_space))
      {
        try
        {
          std::cout<<"trying to get transform \n";
          transformStamped = tfBuffer.lookupTransform(tf_robot_frame, tf_depth_frame,ros::Time(5.0));
        } 
        catch (tf2::TransformException &ex) 
        {
          ROS_WARN("%s", ex.what());
          error = true;
          //ros::Duration(1.0).sleep();
        }
        //if(error == false)
        {
          //tf_in = true;
          robot_space = tf2::transformToEigen(transformStamped);
          Eigen::Isometry3d mat = tf2::transformToEigen(transformStamped);
          robot_frame = mat.matrix();
          //std::cout<<"SUCCESS Transform"<<tf_robot_frame<<" \n";
        }      
      }
      tf_in = true;
      std::cout<<"SUCCESS Transform"<<tf_robot_frame<<" \n";
    }

    //icp matrix hard coded
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
      std::ifstream icpfile ("/home/altair/catkin_abb/src/transform_pcl/config/icp.txt");
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