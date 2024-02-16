/*
This code is exactly the same as the package depth_interface
The ony difference is to be used only for calibration because it's a service
it uses tf to get the transform camera to robot space
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
#include <scene_calibration/PointsDepthMap.h>
#include <unity_msgs/AffineDepthMap.h>

static const std::string OPENCV_WINDOW = "Image window";

class ServerPoints
{
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber dm_sub_;
    typedef image_transport::SubscriberFilter ImageSubscriber;
    std::string tf_robot_frame;
    std::string tf_depth_frame;
    std::string calibration_folder;
    std::string file_recorded;
    std::string name_recording;
    ros::ServiceServer service_points;
    ImageSubscriber depth_rgb_sub_;
    ImageSubscriber depth_sub_;
    ros::Subscriber sub_affine_dm;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer< MySyncPolicy > sync;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::Buffer tfBuffer_rgb;
    tf2_ros::TransformListener tfListener;
    tf2_ros::TransformListener tfListener_rgb;
    k4a::playback handle;
    k4a::calibration k4aCalibration;
    k4a::transformation k4aTransformation;
    std::vector<pcl::PointXYZ> list_poi;
    std::vector<pcl::PointXYZ> pix_depth;
    cv::Mat cv_depth_rgb;
    cv::Mat cv_depth;
    sensor_msgs::PointCloud2 poi_cloud;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped transformStamped_rgb;
    bool tf_in;
    bool tf_rgb;
    Eigen::Affine3d rgb_space;
    Eigen::Affine3d robot_space;
    unity_msgs::AffineDepthMap affine_dm;
    std::vector<pcl::PointXYZ> pts_map;
    double ax;
    double bx;
    double ay;
    double by;
    double az;
    double bz;
    

  public:
    ServerPoints():
    tfListener(tfBuffer),
    tfListener_rgb(tfBuffer_rgb),
    it_(nh_),
    depth_rgb_sub_( it_, "/depth_to_rgb/image_raw", 1),
    depth_sub_( it_, "/depth/image_raw", 1),
    sync( MySyncPolicy(10), depth_rgb_sub_, depth_sub_)
    {
      ros::param::get("~robot_base", tf_robot_frame);
      ros::param::get("~depth_camera", tf_depth_frame);
      ros::param::get("calibration_folder", calibration_folder);
      ros::param::get("file_recorded", file_recorded);
      sync.registerCallback( boost::bind( &ServerPoints::callbackRGBDepth, this, _1, _2) );
      service_points = nh_.advertiseService("server_rgb_points", &ServerPoints::getPointsService, this);
      dm_sub_ = it_.subscribe("/detection/depth_map", 1,&ServerPoints::DepthMapCallback, this);
      readParamsFile();
      name_recording = std::getenv("HOME");
      name_recording = name_recording + file_recorded;
      handle = k4a::playback::open(name_recording.c_str());
      k4aCalibration = handle.get_calibration();
      k4aTransformation = k4a::transformation(k4aCalibration);
      pts_map.resize(0);
      //handle.close();
      tf_in = false;
      tf_rgb = false;
      affine_dm.x.resize(0);
      affine_dm.y.resize(0);
      affine_dm.z.resize(0);
      robot_space = Eigen::Affine3d::Identity();
      cv::namedWindow(OPENCV_WINDOW,cv::WINDOW_NORMAL);
    }
    void callbackRGBDepth(const sensor_msgs::ImageConstPtr& rgbdepth_msg, const sensor_msgs::ImageConstPtr& depth_msg)
    {
      cv_bridge::CvImagePtr cv_bridge_depth_rgb = cv_bridge::toCvCopy(rgbdepth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      cv_bridge::CvImagePtr cv_bridge_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
      cv_depth_rgb = cv_bridge_depth_rgb->image.clone();
      cv_depth = cv_bridge_depth->image.clone();
    }

    void DepthMapCallback(const sensor_msgs::ImageConstPtr& msg_dm)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat depth_map;
        try
        {
          cv::Mat res;
          cv_ptr = cv_bridge::toCvCopy(msg_dm, sensor_msgs::image_encodings::TYPE_32FC1);
          cv::cvtColor(cv_ptr->image,res,cv::COLOR_GRAY2RGB);
          res.convertTo(depth_map, CV_8U, 255.0);
          
          for(int i = 0; i < pts_map.size(); i++)
          {
            cv::Point center(pts_map[i].x, pts_map[i].y);
            circle(depth_map, center,5, cv::Scalar(0, 0, 255), -1);
            //std::cout<<"detection \n";
            //std::cout<<detection_new.poi[i].id<<"\n";
            //std::cout<<detection_new.poi[i].elem<<"\n";
          }
            
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //cv::imshow(OPENCV_WINDOW, depth_map);
        //cv::waitKey(1);
    }

    pcl::PointXYZ getDepthFromRGB(cv::Mat depth_rgb, cv::Mat depth, pcl::PointXYZ p)
    {
      std::vector<pcl::PointXYZ> elem;
      elem.resize(0);
      //get point from depth image
      pcl::PointXYZ p_tmp;
      float x = p.x;
      float y = p.y;
      unsigned short val = depth_rgb.at<unsigned short>(static_cast<int>(y),static_cast<int>(x));
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
      if(suc == true)
      {
        pixel_depth.x = pixel_dest.xy.x;
        pixel_depth.y = pixel_dest.xy.y;
        pixel_depth.z = d;
      }
      return pixel_depth;
    }

    void listenTransform()
    {
      bool error_in = false;
      bool error_rgb = false;
      Eigen::Affine3d cmp = Eigen::Affine3d::Identity();
      
      //compare transformation matrix, much more robust than a boolean to test if the transform has been handled
      while(cmp.isApprox(robot_space))
      {
        try
        {
          transformStamped = tfBuffer.lookupTransform(tf_robot_frame, tf_depth_frame,ros::Time(0));
        } 
        catch (tf2::TransformException &ex) 
        {
          error_in = false;
          ROS_WARN("%s", ex.what());
          ros::Duration(1.0).sleep();
        }
        //if(error_in == false)
        {
          //cout<<"transform base master\n";
          tf_in = true;
          robot_space = tf2::transformToEigen(transformStamped);
        }
      }
      cout<<"success transform base\n";
      tf_in = true;
    }

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

    // fake the creation of point cloud to only get the point we are interested in, then we applu transform to rgb space then to robot frame
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

      pcl::PointXYZ final_pt;
      
      //align to robot frame because the point comes from depth frame
      if(tf_in)
      {
        final_pt = pcl::transformPoint(pt_trans,robot_space);
      }

      return final_pt;
    }
    //Transform PointXYZ from to the depth map perspective
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

    bool getPointsService(scene_calibration::PointsDepthMap::Request& req, scene_calibration::PointsDepthMap::Response& res)
    {
      if(tf_in == false)
      {
        listenTransform();
      }
      pts_map.resize(0);
      
      for(int i = 0; i < req.poi_rgb.size(); i++)
      {
        
        pcl::PointXYZ p;
        p.x = req.poi_rgb[i].x;
        p.y = req.poi_rgb[i].y;
        p.z = req.poi_rgb[i].z;
        pcl::PointXYZ pixel_res = getDepthFromRGB(cv_depth_rgb,cv_depth,p);
        //generate a point XYZ and apply tf
        pcl::PointXYZ pt_depth = generatePointCloudPOI(cv_depth, pixel_res);
        pcl::PointXYZ final_pt = genPointDepthMap(pt_depth);
        pts_map.push_back(final_pt);
      }
      
      res.poi_depth_map.resize(0);
      for(int i = 0; i < pts_map.size(); i++)
      {
        geometry_msgs::Point p;
        p.x = pts_map[i].x;
        p.y = pts_map[i].y;
        p.z = pts_map[i].z;
        res.poi_depth_map.push_back(p);
      }
      return true;
    }

    void readParamsFile()
    {
      std::string line;
      std::string home = std::getenv("HOME");
      std::string name_file = calibration_folder + "params.txt";
      std::ifstream icpfile(name_file);
      if(icpfile.is_open())
      {
        getline(icpfile,line);
        ax = std::stod(line); 
        getline(icpfile,line);
        bx = std::stod(line);
        getline(icpfile,line);
        ay = std::stod(line);
        getline(icpfile,line);
        by = std::stod(line);
        getline(icpfile,line);
        az = std::stod(line);
        getline(icpfile,line);
        bz = std::stod(line);
        icpfile.close();
      }
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "points_depthmap");
  ServerPoints sp;
  ROS_INFO("Ready to get some points...");
  ros::spin();

  return 0;
}