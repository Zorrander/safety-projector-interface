#include <ros/ros.h>
// PCL specific includes
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <scene_calibration/PointsDepthMap.h>
#include <unity_msgs/InterfacePOI.h>
#include <unity_msgs/ElementUI.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <unity_msgs/poiPCL.h>
#include <limits>
#include <unity_msgs/AffineDepthMap.h>

using namespace sensor_msgs;
using namespace message_filters;
static const std::string OPENCV_WINDOW = "Image window";

class FusionPcl
{
    private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    cv_bridge::CvImagePtr cv_ptr;
    ros::Publisher pub_merged_pcl;
    ros::Publisher pub_poi_pcl;
    ros::Publisher pub_affine_dm;
    image_transport::Publisher pub_dm;
    ros::Subscriber single_master;
    ros::Subscriber single_sub;
    ros::Subscriber sub_poi;
    ros::Subscriber sub_poi_rgb;
    ros::ServiceClient spawnClient;
    int number_cam;
    std::string name_sub_master;
    std::string name_sub_sub;
    std::string calibration_folder;
    //message_filters::Subscriber<sensor_msgs::PointCloud2> master_sub;
    //message_filters::Subscriber<sensor_msgs::PointCloud2> sub_sub;
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    pcl::console::TicToc tic;
    Eigen::Matrix4d flip_matrix;
    float crop_min_x;
    float crop_min_y;
    float crop_max_x;
    float crop_max_y;
    float crop_max_z;
    float crop_min_z;
    double ax;
    double bx;
    double ay;
    double by;
    double az;
    double bz;
    std::vector<pcl::PointXYZ> poi_depth;
    pcl::PCLPointCloud2 pcl_poi;
    pcl::PCLPointCloud2 pcl_master;
    int count;
    float resize;
    bool calib;
    
  public:
    FusionPcl():
    it_(nh_)
    {
      //master_sub.subscribe(nh_, "/kinect_master/cloud_robot_frame", 1);
      //sub_sub.subscribe(nh_, "/kinect_sub/cloud_robot_frame", 1);
      ros::param::get("~number_cam", number_cam);
      ros::param::get("~sub_pcl_master", name_sub_master);
      ros::param::get("~sub_pcl_sub", name_sub_sub);
      ros::param::get("calibration_folder", calibration_folder);
      single_master = nh_.subscribe(name_sub_master, 1, &FusionPcl::callbackFrameMaster,this);
      if(number_cam > 1)
      {
        single_sub = nh_.subscribe(name_sub_sub, 1, &FusionPcl::callbackFrameSub,this);
      }
      
      //sub_poi = nh_.subscribe("/kinect_sub/poi_robot_frame", 1, &FusionPcl::poiCb,this); // TO DO
      pub_dm = it_.advertise("/detection/depth_map", 1);
      //pub_poi_pcl = nh_.advertise<unity_msgs::poiPCL> ("/detection/poi_pcl", 1);
      pub_merged_pcl = nh_.advertise<sensor_msgs::PointCloud2> ("/pcl_fusion/fused_pcl_kinects", 1);
      //pub_affine_dm = nh_.advertise<unity_msgs::AffineDepthMap> ("/pcl_fusion/affine_dm", 1);
      //sync.reset(new Sync(MySyncPolicy(10), master_sub, sub_sub));      
      //sync->registerCallback(boost::bind(&FusionPcl::fuseAndDetectCb, this, _1, _2));
      count = 0;
      resize = 0.7;
      cv::namedWindow(OPENCV_WINDOW,cv::WINDOW_NORMAL);
      calib = calibrationDepthMap();
      if(calib)
      {
        readCalibrationFile();
        readParamsDM();
      }
      else
      {
        crop_max_x = -5000;
        crop_max_y = -5000;
        crop_min_x = 5000;
        crop_min_y = 5000;
        crop_min_z = 0;
        crop_max_z = -5000;
      }
    }
    ~FusionPcl()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }

    void callbackFrameMaster(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
      
      pcl_conversions::toPCL(*cloud_msg, pcl_master);
      if(number_cam == 1)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_master(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_master,*cloud_master);
        if(!calib)
        {
          getExtremeValues(cloud_master);
          if(count == 250)
          {
            
            resizeDepthMap();
            writeExtremeValues();
            writeCalibrationParams();
            calib = true;
            std::cout<<"calibration depthmap done \n";
          }
          count++;
        }
        genDepthFromPcl(cloud_master);
        sensor_msgs::PointCloud2 cloud_publish;
        pcl::toROSMsg(*cloud_master,cloud_publish);
        cloud_publish.header = cloud_msg->header;
        pub_merged_pcl.publish(cloud_publish);
      }
    }

    void callbackFrameSub(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_master(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sub(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_poi(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

      pcl::fromPCLPointCloud2(pcl_master,*cloud_master);

      pcl::PCLPointCloud2 pcl_pc3;
      pcl_conversions::toPCL(*cloud_msg, pcl_pc3);
      pcl::fromPCLPointCloud2(pcl_pc3,*cloud_sub);

      //tic.tic();
      *final_cloud = *cloud_master + *cloud_sub;
      if(!calib)
      {
        getExtremeValues(final_cloud);
        if(count == 250)
        {
          resizeDepthMap();
          writeExtremeValues();
          writeCalibrationParams();
          calib = true;
          std::cout<<"calibrated !\n";
        }
        std::cout<<count<<"\n";
        count++;
      }
      genDepthFromPcl(final_cloud);

      sensor_msgs::PointCloud2 cloud_publish;
      pcl::toROSMsg(*final_cloud,cloud_publish);
      cloud_publish.header = cloud_msg->header;
      pub_merged_pcl.publish(cloud_publish);
    }

    void getExtremeValues(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
      float min_x = 0;
      float max_x = 0;
      float min_y = 0;
      float max_y = 0;
      float min_z = 0;
      float max_z = 0;
      float px;
      float py;
      float pz;
      std::vector<float> min_max_values;
      for (int i=0; i< cloud->points.size();i++)
      {
        if (cloud->points[i].z == cloud->points[i].z)
        {
            px = cloud->points[i].x * 1000.0;// *1000.0;
            py = cloud->points[i].y * 1000.0;// *1000.0*-1; //revert image because it's upside down for display
            pz = cloud->points[i].z * 1000.0;
            //std::cout<<"z : "<<pz<<"\n";
            if(px < min_x)
            {
              min_x = px;
            }
            if(px > max_x)
            {
              max_x = px;
            }
            if(py < min_y)
            {
              min_y = py;
            }
            if(py > max_y)
            {
              max_y = py;
            }
            if(pz < min_z)
            {
              min_z = pz;
            }
            if(pz > max_z)
            {
              max_z = pz;
            }
        }
      }
      if(min_x < crop_min_x)
      {
        crop_min_x = min_x;
      }
      if(min_y < crop_min_y)
      {
        crop_min_y = min_y;
      }
      if(max_x > crop_max_x)
      {
        crop_max_x = max_x;
      }
      if(max_y > crop_max_y)
      {
        crop_max_y = max_y;
      }
      if(min_z < crop_min_z)
      {
        crop_min_z = min_z;
      }
      if(max_z > crop_max_z)
      {
        crop_max_z = max_z;
      }
      //std::cout<<"min z : "<<crop_min_z<<"\n";
      //std::cout<<"max z : "<<crop_max_z<<"\n";
    }

    void genDepthFromPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
      cv::Mat cv_image = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
      const float bad_point = std::numeric_limits<float>::quiet_NaN();
      sensor_msgs::ImagePtr msg_dm;
      tic.tic();
      
      int pixel_pos_x;
      int pixel_pos_y;
      float pixel_pos_z;
      float px;
      float py;
      float pz;
      float ref;

      ax = (static_cast<double>(1024))/(crop_max_x-crop_min_x); //1024 image width
      bx = 0 - (ax*crop_min_x);
      ay = (static_cast<double>(1024))/(crop_max_y-crop_min_y); //1024 image height
      by = 0 - (ay*crop_min_y);
      az = (static_cast<double>(1000))/(crop_max_z-crop_min_z);
      bz = 0 - (az*crop_min_z);


      for (int i=0; i< cloud->points.size();i++)
      {
        px = cloud->points[i].x * 1000.0;//revert image because it's upside down for display
        py = cloud->points[i].y * 1000.0;
        pz = cloud->points[i].z *1000.0;
        if(px > crop_max_x || px < crop_min_x || py > crop_max_y || py < crop_min_y)
        {
          cloud->points[i].z = bad_point;
        }
        if (cloud->points[i].z == cloud->points[i].z)
        {
            pixel_pos_x = (int) (ax * px + bx);
            pixel_pos_y = (int) (ay * py + by);
            pixel_pos_z = (az * pz + bz);
            pixel_pos_z = pixel_pos_z/1000.0;     //important here
            ref = cv_image.at<float>(pixel_pos_y,pixel_pos_x); 
            //std::cout<<"test : "<<ref<<"\n";
            // pick highest point - simplistic way of dealing with occlusions
            // It avoid to see the floor instead of a person or seeing the table instead of an arm on top of the table
            if(pixel_pos_z > ref)
            {
              //std::cout<<"high value : "<<pixel_pos_z<<"\n";
              cv_image.at<float>(pixel_pos_y,pixel_pos_x) = pixel_pos_z;
            }
            
            //cv_image.at<float>(pixel_pos_y,pixel_pos_x) = pixel_pos_z;
        }
      }
      //std::cout<<"time : "<<tic.toc()<<"ms\n";
      //std::cout<<"poi_interface_pcl : "<<poi_interface_pcl.size()<<"\n";
      msg_dm = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_32FC1, cv_image).toImageMsg();
      pub_dm.publish(msg_dm);
      
      //cv::imshow(OPENCV_WINDOW, cv_image);
      //cv::waitKey(1);
    }

    void resizeDepthMap()
    {
      crop_min_x = crop_min_x * resize;
      crop_max_x = crop_max_x * resize;
      crop_min_y = crop_min_y * resize;
      crop_max_y = crop_max_y * resize;
    }

    void writeExtremeValues()
    {
      std::ofstream ofile;
      std::string home = std::getenv("HOME");
      std::string name_file = home + calibration_folder + "extreme_values.txt";
      ofile.open(name_file, std::ios::app);
      ofile << crop_min_x << std::endl;
      ofile << crop_max_x << std::endl;
      ofile << crop_min_y << std::endl;
      ofile << crop_max_y << std::endl;
      ofile << crop_min_z << std::endl;
      ofile << crop_max_z << std::endl;
      ofile.close();
    }

    void writeCalibrationParams()
    {
      std::string home = std::getenv("HOME");
      std::string name_file = home + calibration_folder + "params.txt";
      double tax = (static_cast<double>(1024))/(crop_max_x-crop_min_x); //1024 image width
      double tbx = 0 - (tax*crop_min_x);
      double tay = (static_cast<double>(1024))/(crop_max_y-crop_min_y); //1024 image height
      double tby = 0 - (tay*crop_min_y);
      double taz = (static_cast<double>(1000))/(crop_max_z-crop_min_z);
      double tbz = 0 - (taz*crop_min_z);
      std::ofstream ofile;
      ofile.open(name_file, std::ios::app);
      ofile << tax << std::endl;
      ofile << tbx << std::endl;
      ofile << tay << std::endl;
      ofile << tby << std::endl;
      ofile << taz << std::endl;
      ofile << tbz << std::endl;
      ofile.close();
    }

    bool calibrationDepthMap()
    {
      bool calib = false;
      std::string home = std::getenv("HOME");
      std::string name_file = home + calibration_folder + "extreme_values.txt";
      std::ifstream icpfile (name_file);
      if(icpfile.is_open())
      {
        calib = true;
      }
      return calib;
    }

    void readParamsDM()
    {
      std::string line;
      std::string home = std::getenv("HOME");
      std::string name_file = home + calibration_folder + "params.txt";
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

    void readCalibrationFile()
    {
      std::string line;
      std::string home = std::getenv("HOME");
      std::string name_file = home + calibration_folder + "extreme_values.txt";
      std::ifstream icpfile(name_file);
      if(icpfile.is_open())
      {
        getline(icpfile,line);
        crop_min_x = std::stof(line);
        getline(icpfile,line);
        crop_max_x = std::stof(line);
        getline(icpfile,line);
        crop_min_y = std::stof(line);
        getline(icpfile,line);
        crop_max_y = std::stof(line);
        getline(icpfile,line);
        crop_min_z = std::stof(line);
        getline(icpfile,line);
        crop_max_z = std::stof(line);
        icpfile.close();
      }
    }
}; 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_fuse");
  FusionPcl fpcl;
  ros::spin();
  return 0;
}