#include <ros/ros.h>
// PCL specific includes
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/visualization/cloud_viewer.h>


using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg_master, const sensor_msgs::PointCloud2ConstPtr& cloud_msg_sub)
{
  // Create a container for the data.
  std::cout<<"Merged !\n";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_master(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sub(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_msg_master, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud_master);
  pcl::PCLPointCloud2 pcl_pc3;
  pcl_conversions::toPCL(*cloud_msg_sub, pcl_pc3);
  pcl::fromPCLPointCloud2(pcl_pc3,*cloud_sub);

  *final_cloud = *cloud_master + *cloud_sub;

  sensor_msgs::PointCloud2 cloud_publish;
  pcl::toROSMsg(*final_cloud,cloud_publish);
  cloud_publish.header = cloud_msg_master->header;
      
  pub.publish(cloud_publish);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_fuse");
  ros::NodeHandle nh_;
  pub = nh_.advertise<sensor_msgs::PointCloud2> ("/fused_pcl", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> master_sub(nh_, "/kinect_master/cloud_robot_frame", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_sub(nh_, "/kinect_sub/cloud_robot_frame", 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
  //typedef Synchronizer<MySyncPolicy> Sync;
  //boost::shared_ptr<Sync> sync;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), master_sub, sub_sub);
  
  
  //sync.reset(new Sync(MySyncPolicy(10), master_sub, sub_sub));      
  sync.registerCallback(boost::bind(&cloud_cb, _1, _2));
  
  ros::spin();
  return 0;
}