/*
In this code, we transform RGB coordinates to the depthmap. It uses the Azure Kinect API and uses the video sample acquired during calibration to extract intrinsic camera parameters of the kinect.
First we must find where is the RGB point in the depth image -> getDepthFromRGB()
Once we have the point in the depth image (x,y,depth_value), we get the 3D point by generating a single point pointcloud and transforming it to robot base -> generatePointCloudPOI()
Then, we project this 3D point aligned with the robot frame to the depthmap with the depthmap parameters -> genPointDepthMap
*/

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

static const std::string OPENCV_WINDOW = "Image window";
using namespace std;

class DepthInterface
{
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    typedef image_transport::SubscriberFilter ImageSubscriber;
    std::string tf_robot_frame;
    std::string tf_depth_frame;
    std::string tf_rgb_frame;
    std::string calibration_folder;
    ros::ServiceServer service_points;
    ImageSubscriber depth_rgb_sub_;
    ImageSubscriber depth_sub_;
    ros::Subscriber sub_hand_poi;
    ros::Subscriber sub_poi;
    ros::Publisher pub_poi;
    ros::Publisher pub_poi_pcl;
    ros::Publisher pub_hand_tracker_dm;
    image_transport::Subscriber dm_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer< MySyncPolicy > sync;
    k4a::playback handle;
    k4a::calibration k4aCalibration;
    k4a::transformation k4aTransformation;
    std::vector<pcl::PointXYZ> list_poi;
    std::vector<pcl::PointXYZ> pix_depth;
    std::string name_recording;
    std::string file_recorded;
    cv::Mat cv_depth_rgb;
    cv::Mat cv_depth;
    sensor_msgs::PointCloud2 poi_cloud;
    bool tf_in;
    bool depth_images;
    Eigen::Affine3d robot_space;
    unity_msgs::InterfacePOI list_points;
    unity_msgs::poiPCL list_points_hand;
    double ax;
    double bx;
    double ay;
    double by;
    double az;
    double bz;
    std::string name_tf_robot;
    geometry_msgs::Transform rgb_to_base;
    string hand_tracking_rgb_coordinates, hand_tracking_dm_coordinates;

  public:
    DepthInterface(std::string name_drgb, std::string name_d):
    it_(nh_),
    depth_rgb_sub_( it_, name_drgb, 1),
    depth_sub_( it_, name_d, 1),
    sync( MySyncPolicy(10), depth_rgb_sub_, depth_sub_)
    {
      ros::param::get("robot_base", tf_robot_frame);
      ros::param::get("calibration_folder", calibration_folder);
      ros::param::get("file_recorded", file_recorded);
      ros::param::get("name_transform_robot_space", name_tf_robot);
      sync.registerCallback( boost::bind( &DepthInterface::callbackRGBDepth, this, _1, _2) );
      
      
      ros::param::get("hand_tracking_rgb_coordinates", hand_tracking_rgb_coordinates);
      sub_hand_poi = nh_.subscribe(hand_tracking_rgb_coordinates, 1, &DepthInterface::handTrackerCallback,this);

      ros::param::get("hand_tracking_dm_coordinates", hand_tracking_dm_coordinates);
      pub_hand_tracker_dm = nh_.advertise<unity_msgs::poiPCL> (hand_tracking_dm_coordinates, 1);

      sub_poi = nh_.subscribe("/interface_poi/buttons", 1, &DepthInterface::poiCallback,this);
      pub_poi = nh_.advertise<unity_msgs::InterfacePOI> ("/depth_interface/poi_depthmap", 1);
      //dm_sub_ = it_.subscribe("/detection/depth_map", 1,&DepthInterface::callbackDepthMap, this);
      pub_poi_pcl = nh_.advertise<sensor_msgs::PointCloud2> ("/depth_interface/poi_pcl", 1);
      robot_space = Eigen::Affine3d::Identity();
      tf_in = false;
      readParamsFile();
      initTransformToBase();
      name_recording = file_recorded;
      handle = k4a::playback::open(name_recording.c_str());
      k4aCalibration = handle.get_calibration();
      k4aTransformation = k4a::transformation(k4aCalibration);
      depth_images = false;
      list_points.poi.resize(0);
    }
    //get the camera robot transform
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
      robot_space = tf2::transformToEigen(rgb_to_base);
      
      tf_in = true;
    }

    //Display hand locaiton on depthmap for debugging
    void callbackDepthMap(const sensor_msgs::ImageConstPtr& msg_dm)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg_dm, sensor_msgs::image_encodings::TYPE_32FC1);
          
          for(int i = 0; i < list_points_hand.pts.size(); i++)
          {
            cv::Point center(list_points_hand.pts[i].x, list_points_hand.pts[i].y);
            circle(cv_ptr->image, center,5, cv::Scalar(255, 255, 255), -1);
          }
            
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(1);
    }
    //get the rgb_to_depth and depth images
    void callbackRGBDepth(const sensor_msgs::ImageConstPtr& rgbdepth_msg, const sensor_msgs::ImageConstPtr& depth_msg)
    {
      cv_bridge::CvImagePtr cv_bridge_depth_rgb = cv_bridge::toCvCopy(rgbdepth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      cv_bridge::CvImagePtr cv_bridge_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      cv_depth_rgb = cv_bridge_depth_rgb->image.clone();
      cv_depth = cv_bridge_depth->image.clone();
      //cout<<"depth rgb :"<<cv_depth_rgb.size<<"\n";
      //cout<<"depth :"<<cv_depth.size<<"\n";
      depth_images = true;

    }
    //callback that transform RGB point of the hands to their coordinates in the depthmap
    void handTrackerCallback(const unity_msgs::poiPCLConstPtr& msg)
    {

      pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      list_points_hand.pts.clear();
      if(!tf_in)
      {
        initTransformToBase();
      }
      else
      {
        
        for(int i = 0; i < msg->pts.size(); i++)
        {
          geometry_msgs::Point current_elem;
          pcl::PointXYZ p;
          p.x = static_cast<float>(msg->pts[i].x);
          p.y = static_cast<float>(msg->pts[i].y);
          p.z = static_cast<float>(msg->pts[i].z);
          if(depth_images)
          {
            pcl::PointXYZ pixel_res = getDepthFromRGB(cv_depth_rgb,cv_depth,p);
            pcl::PointXYZ pt_depth = generatePointCloudPOI(cv_depth, pixel_res);
            pcl::PointXYZ final_pt = genPointDepthMap(pt_depth);
            
            current_elem.x = static_cast<double>(final_pt.x);
            current_elem.y = static_cast<double>(final_pt.y);
            current_elem.z = static_cast<double>(final_pt.z);
            list_points_hand.pts.push_back(current_elem);
          }
        }
        list_points_hand.header = msg->header;
        //publish hand coordinates in the depthmap
        pub_hand_tracker_dm.publish(list_points_hand);
      }
    }

    //get rgb points from interface and find them in the global depth map
    void poiCallback(const unity_msgs::InterfacePOIConstPtr& msg)
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
    }

   // get the depth value of the interface pixels given their location in the RGB space
    pcl::PointXYZ getDepthFromRGB(cv::Mat depth_rgb, cv::Mat depth, pcl::PointXYZ p)
    {
      std::vector<pcl::PointXYZ> elem;
      elem.resize(0);
      //get point from depth image
      pcl::PointXYZ p_tmp;
      float x = p.x;
      float y = p.y;
      unsigned short val = depth_rgb.at<unsigned short>(static_cast<int>(y),static_cast<int>(x));
      //cout<<"marker gen 2\n";
      float d = static_cast<float>(val);
      p_tmp.x = x;
      p_tmp.y = y;
      p_tmp.z = d;
      //get corresponding point in depth frame
      k4a_float2_t pixel_source;
      k4a_float2_t pixel_dest;  
      float d_;
      pixel_source.xy.x = p_tmp.x;
      pixel_source.xy.y = p_tmp.y;
      d_ = p_tmp.z;
      
      bool suc = k4aCalibration.convert_2d_to_2d(pixel_source,d_,K4A_CALIBRATION_TYPE_COLOR,K4A_CALIBRATION_TYPE_DEPTH,&pixel_dest);
      pcl::PointXYZ pixel_depth;
      //cout<<"marker gen 3\n";
      if(suc == true)
      {
        pixel_depth.x = pixel_dest.xy.x;
        pixel_depth.y = pixel_dest.xy.y;
        pixel_depth.z = d;
      }
      return pixel_depth;
    }

    // generate the 3D point(s) of the RGB coordinates we are interested in (hands, interface buttons)
    pcl::PointXYZ generatePointCloudPOI(cv::Mat depth_image, pcl::PointXYZ pix)
    {
      sensor_msgs::PointCloud2Ptr pt_cloud(new sensor_msgs::PointCloud2);
      float zero_d = 0;
      unsigned short z = static_cast<unsigned short> (zero_d);
      for(int i = 0; i < depth_image.rows;i++)
      {
        for(int j = 0; j < depth_image.cols;j++)
        {
          depth_image.at<unsigned short> (j,i) = z;
        }
      }

      int x = static_cast<int>(pix.x);
      int y = static_cast<int>(pix.y);
      float max_de = pix.z;
      unsigned short m_ = static_cast<unsigned short> (max_de);
      depth_image.at<unsigned short> (y,x) = m_;
      pcl::PointXYZ pt;

      k4a::image image_k = k4a::image::create_from_buffer(K4A_IMAGE_FORMAT_DEPTH16,depth_image.cols,depth_image.rows,(int)depth_image.step,depth_image.data,depth_image.step * depth_image.rows,nullptr,nullptr);
      k4a::image pc = k4aTransformation.depth_image_to_point_cloud(image_k,K4A_CALIBRATION_TYPE_DEPTH);
      pt = fillPointCloud(pc,pt_cloud);
      
      return pt;
    }

    // fill point cloud with only the point we are interested in, then we apply transform to robot frame.
    pcl::PointXYZ fillPointCloud(const k4a::image& pointcloud_image, sensor_msgs::PointCloud2Ptr& point_cloud)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointXYZ pt_trans;
      point_cloud->height = pointcloud_image.get_height_pixels();
      point_cloud->width = pointcloud_image.get_width_pixels();
      point_cloud->is_dense = false;
      point_cloud->is_bigendian = false;

      const size_t point_count = pointcloud_image.get_height_pixels() * pointcloud_image.get_width_pixels();

      sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
      pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

      sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

      pcd_modifier.resize(point_count);

      const int16_t* point_cloud_buffer = reinterpret_cast<const int16_t*>(pointcloud_image.get_buffer());
      //std::cout<<"entering CONSTRUCTION...\n";
      for (size_t i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z)
      {
        float z = static_cast<float>(point_cloud_buffer[3 * i + 2]);

        if (z <= 0.0f)
        {
          *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
          constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
          *iter_x = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 0]);
          *iter_y = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 1]);
          *iter_z = kMillimeterToMeter * z;
          //std::cout<<"x "<<*iter_x<<" y "<<*iter_y<<" z "<<*iter_z<<"\n";
          pt_trans.x = *iter_x;
          pt_trans.y = *iter_y;
          pt_trans.z = *iter_z;
        }
      }

      pcl::PointXYZ tmp_res;
      pcl::PointXYZ final_pt;
      
      if(tf_in)
      {
        final_pt = pcl::transformPoint(pt_trans,robot_space);
      }

      return final_pt;
    }

    //Transform PointXYZ from point cloud to the depth map perspective
    pcl::PointXYZ genPointDepthMap(pcl::PointXYZ point_d)
    {
      double px;
      double py;
      double pz;
      int pixel_pos_x;
      int pixel_pos_y;
      float pixel_pos_z;
      px = point_d.x * 1000.0;
      py = point_d.y * 1000.0;
      pz = point_d.z * 1000.0;
      pixel_pos_x = (int) (ax * px + bx);
      pixel_pos_y = (int) (ay * py + by);
      pixel_pos_z = (az * pz + bz);
      pixel_pos_z = pixel_pos_z/1000.0;
      pcl::PointXYZ p;
      p.x = static_cast<float>(pixel_pos_x);
      p.y = static_cast<float>(pixel_pos_y);
      p.z = static_cast<float>(pixel_pos_z);

      return p;
    }
    //get depthmap parameters
    void readParamsFile()
    {
      std::string line;
      std::string name_file = calibration_folder + "params.txt";
      std::ifstream calibfile(name_file);
      if(calibfile.is_open())
      {
        getline(calibfile,line);
        ax = std::stod(line); 
        getline(calibfile,line);
        bx = std::stod(line);
        getline(calibfile,line);
        ay = std::stod(line);
        getline(calibfile,line);
        by = std::stod(line);
        getline(calibfile,line);
        az = std::stod(line);
        getline(calibfile,line);
        bz = std::stod(line);
        calibfile.close();
      }
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_depthmap");
  std::string d;
  std::string drgb;

  ros::param::get("topic_depth", d);
  ros::param::get("topic_depth_to_rgb", drgb);
  DepthInterface sp(drgb,d);
  ros::spin();

  return 0;
}