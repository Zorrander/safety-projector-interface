/*
Package generating the depthmap with the PCL coming from the master (and slave if any) camera
*/
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

#include <k4a/k4a.hpp>

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
    ros::Publisher pub_hand_tracker_dm;
    ros::ServiceClient spawnClient;
    int number_cam;
    std::string name_sub_master;
    std::string name_sub_sub;
    std::string calibration_folder;
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    pcl::console::TicToc tic;
    Eigen::Matrix4d flip_matrix;
    cv::Mat baseline;
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
    cv::Mat cv_image;
    double bz;
    std::vector<pcl::PointXYZ> poi_depth;
    pcl::PCLPointCloud2 pcl_poi;
    pcl::PCLPointCloud2 pcl_master;
    int count;
    int count_bl;
    float resize;
    bool calib;
    bool bl;

    unity_msgs::poiPCL list_points_hand;
    bool tf_in;

  public:
    FusionPcl():
    it_(nh_)
    {
      //getting ros params
      ros::param::get("number_cam", number_cam);
      ros::param::get("topic_pcl_master", name_sub_master);
      ros::param::get("topic_pcl_sub", name_sub_sub);
      ros::param::get("calibration_folder", calibration_folder);
      ROS_INFO("Looking for calibration data at: %s", ("/camera1"+name_sub_master).c_str());

      pub_hand_tracker_dm = nh_.advertise<unity_msgs::poiPCL> ("/hand_tracking/dm/coordinates", 1);
      single_master = nh_.subscribe("/camera1"+name_sub_master, 1, &FusionPcl::callbackFrameMaster,this);
      //if more than one camera, also subscribe to the slave one - this will be where we generate the depthmap
      if(number_cam > 1)
      {
        single_sub = nh_.subscribe(name_sub_sub, 1, &FusionPcl::callbackFrameSub,this);
        std::cout<<"number cam "<<number_cam<<"\n";
      }
      cv_image = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
      pub_dm = it_.advertise("/detection/depth_map", 1);
      pub_merged_pcl = nh_.advertise<sensor_msgs::PointCloud2> ("/pcl_fusion/fused_pcl_kinects", 1);
      baseline = cv::Mat(1024, 1024, CV_32FC1,cv::Scalar(std::numeric_limits<float>::min()));
      count = 0;
      resize = 0.7;
      bl = false;
      count_bl = 0;
      cv::namedWindow(OPENCV_WINDOW,cv::WINDOW_NORMAL);
      //get depthmap calibration params if they exist
      calib = readCalibrationFile();
      if(calib)
      {
        ROS_INFO("Found depthmap calibration");
        readParamsDM();
        bl = true;
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

    void performCalibration(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_master) {
      //get pointcloud boundaries for 250 steps
      if(count < 250)
      {
        getExtremeValues(cloud_master);
      }
      if(count == 250)
      {
        resizeDepthMap();
        writeExtremeValues();
        writeCalibrationParams();
        std::cout<<"calibration depthmap done \n";
      }
      //now acquire the baseline that will be used for detection
      if(count > 325)
      {
        std::string name_bl = calibration_folder + "baseline";
        writeRawImage(baseline,name_bl);
        bl = true;
        calib = true;
        std::cout<<"baseline done \n";
      }
      count++;
    }
    
    //subscriber to master camera
    void callbackFrameMaster(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
      ROS_INFO("callbackFrameMaster");
      // Convert ROS PointCloud2 message to PCL PointCloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_master(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*cloud_msg, *cloud_master);

      //if only one camera, check calibration ang generate depthmap
      if(number_cam == 1)
      {
        //if there is no calibration
        if(!calib)
        {
          performCalibration(cloud_master);
        }
        //generate and publish depthmap
        genDepthFromPcl(cloud_master);
        //send pcl for display - to be removed later
        sensor_msgs::PointCloud2 cloud_publish;
        pcl::toROSMsg(*cloud_master,cloud_publish);
        cloud_publish.header = cloud_msg->header;
        pub_merged_pcl.publish(cloud_publish);
      }
    }
    //subscriber to slave camera
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
      //merging pointclouds from master and slave camera
      *final_cloud = *cloud_master + *cloud_sub;
      //check calibration
      if(!calib)
      {
        if(count < 250)
        {
          getExtremeValues(final_cloud);
        }
        if(count == 250)
        {
          resizeDepthMap();
          writeExtremeValues();
          writeCalibrationParams();
          std::cout<<"calibration depthmap done \n";
        }
        if(count > 350)
        {
          std::string name_bl = calibration_folder + "baseline";
          writeRawImage(baseline,name_bl);
          bl = true;
          calib = true;
          std::cout<<"baseline done \n";
        }
        count++;
      }
      //generating depthmap
      genDepthFromPcl(final_cloud);
      //send pcl for display - to be removed later
      sensor_msgs::PointCloud2 cloud_publish;
      pcl::toROSMsg(*final_cloud,cloud_publish);
      cloud_publish.header = cloud_msg->header;
      pub_merged_pcl.publish(cloud_publish);
    }

    //get the boundaries of the pointclouds. Since it's a noisy process, we want to get the extremes values to avoid outliers.
    // These paramas will serve for the linear transform that project the 3D points to the 2D depth image.
    //We also get the crop_x, crop_y to resize the depthmap at the end of calibration. By default the depthmap is accurate but there are a lot of black area all around.
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
    }

    //generate the depthmap
    void genDepthFromPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
      const float bad_point = std::numeric_limits<float>::quiet_NaN();
      sensor_msgs::ImagePtr msg_dm;
      
      int pixel_pos_x;
      int pixel_pos_y;
      float pixel_pos_z;
      float px;
      float py;
      float pz;
      float ref;
      //get parameters for linear projection of 3D points on 2D plane
      ax = (static_cast<double>(1024))/(crop_max_x-crop_min_x); //1024 image width
      bx = 0 - (ax*crop_min_x);
      ay = (static_cast<double>(1024))/(crop_max_y-crop_min_y); //1024 image height
      by = 0 - (ay*crop_min_y);
      az = (static_cast<double>(1000))/(crop_max_z-crop_min_z);
      bz = 0 - (az*crop_min_z);

      //entering the pointcloud
      for (int i=0; i< cloud->points.size();i++)
      {
        px = cloud->points[i].x * 1000.0;
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
            // pick highest point - simplistic way of dealing with occlusions
            // It avoid to see the floor instead of a person or seeing the table instead of an arm on top of the table
            if(pixel_pos_z > ref)
            {
              //assigne the depth value to the pixel
              cv_image.at<float>(pixel_pos_y,pixel_pos_x) = pixel_pos_z;
            }
        }
      }

      //if the calibration is not done, this will generate the baseline
      if(!bl && count > 260)
      {
        for(int i = 0; i < cv_image.rows; i++)
        {
          for(int j = 0; j < cv_image.cols; j++)
          {
            float pix_bl = baseline.at<float>(i,j);
            float pix_image = cv_image.at<float>(i,j);
            if(pix_image > pix_bl)
            {
              baseline.at<float>(i,j) = pix_image;
            }
          }
        }
      }

      //publish the depthmap
      msg_dm = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_32FC1, cv_image).toImageMsg();
      pub_dm.publish(msg_dm);
      
      //display the depthmap for debugging
      cv::imshow(OPENCV_WINDOW, cv_image);
      cv::waitKey(1);
    }

    //callback that transform RGB point of the hands to their coordinates in the depthmap
    void handTrackerCallback(const unity_msgs::poiPCLConstPtr& msg)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      list_points_hand.pts.clear();
       for(int i = 0; i < msg->pts.size(); i++)
        {
          geometry_msgs::Point current_elem;
          pcl::PointXYZ p;
          p.x = static_cast<float>(msg->pts[i].x);
          p.y = static_cast<float>(msg->pts[i].y);
          p.z = static_cast<float>(msg->pts[i].z);

          pcl::PointXYZ pixel_res = getDepthFromRGB(p);
          pcl::PointXYZ pt_depth = generatePointCloudPOI(pixel_res);
          pcl::PointXYZ final_pt = genPointDepthMap(pt_depth);
            
          current_elem.x = static_cast<double>(final_pt.x);
          current_elem.y = static_cast<double>(final_pt.y);
          current_elem.z = static_cast<double>(final_pt.z);
          list_points_hand.pts.push_back(current_elem);
          
        }
        list_points_hand.header = msg->header;
        //publish hand coordinates in the depthmap
        pub_hand_tracker_dm.publish(list_points_hand);
      }
    

    // Function to get depth of a point in the point cloud
    float getPointDepth(const PointCloud::Ptr& cloud, const Eigen::Vector3f& point_of_interest) {
        // Create KD tree for fast nearest neighbor search
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

        // Search for the nearest neighbor
        pcl::PointXYZ search_point(point_of_interest[0], point_of_interest[1], point_of_interest[2]);
        int k = 1;  // Number of nearest neighbors to search for
        std::vector<int> point_indices(k);
        std::vector<float> point_distances(k);
        kdtree.nearestKSearch(search_point, k, point_indices, point_distances);

        // Extract depth information from the nearest neighbor
        float depth = cloud->points[point_indices[0]].z;
        return depth;
    }

    // get the depth value of the interface pixels given their location in the RGB space
    pcl::PointXYZ getDepthFromRGB(pcl::PointXYZ p)
    {
      std::vector<pcl::PointXYZ> elem;
      elem.resize(0);
      //get point from depth image
      pcl::PointXYZ p_tmp;
      float x = p.x;
      float y = p.y;
      unsigned short val = cv_image.at<unsigned short>(static_cast<int>(y),static_cast<int>(x));
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
    pcl::PointXYZ generatePointCloudPOI(pcl::PointXYZ pix)
    {
      sensor_msgs::PointCloud2Ptr pt_cloud(new sensor_msgs::PointCloud2);
      float zero_d = 0;
      unsigned short z = static_cast<unsigned short> (zero_d);
      for(int i = 0; i < cv_image.rows;i++)
      {
        for(int j = 0; j < cv_image.cols;j++)
        {
          cv_image.at<unsigned short> (j,i) = z;
        }
      }

      int x = static_cast<int>(pix.x);
      int y = static_cast<int>(pix.y);
      float max_de = pix.z;
      unsigned short m_ = static_cast<unsigned short> (max_de);
      cv_image.at<unsigned short> (y,x) = m_;
      pcl::PointXYZ pt;

      k4a::image image_k = k4a::image::create_from_buffer(K4A_IMAGE_FORMAT_DEPTH16,cv_image.cols,cv_image.rows,(int)cv_image.step,cv_image.data,cv_image.step * cv_image.rows,nullptr,nullptr);
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
          *iter_x = *iter_y = *iter_z = numeric_limits<float>::quiet_NaN();
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
      
      //if(tf_in)
      //{
      //  final_pt = pcl::transformPoint(pt_trans,robot_space);
      //}

      return pt_trans;
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

    //save an depth image at some location. Used to save the baseline.
    bool writeRawImage(const cv::Mat& image, const std::string& filename)
    {
        ofstream file;
        file.open (filename, ios::out|ios::binary);
        if (!file.is_open())
            return false;
        file.write(reinterpret_cast<const char *>(&image.rows), sizeof(int));
        file.write(reinterpret_cast<const char *>(&image.cols), sizeof(int));
        const int depth = image.depth();
        const int type  = image.type();
        const int channels = image.channels();
        file.write(reinterpret_cast<const char *>(&depth), sizeof(depth));
        file.write(reinterpret_cast<const char *>(&type), sizeof(type));
        file.write(reinterpret_cast<const char *>(&channels), sizeof(channels));
        int sizeInBytes = image.step[0] * image.rows;
        file.write(reinterpret_cast<const char *>(&sizeInBytes), sizeof(int));
        file.write(reinterpret_cast<const char *>(image.data), sizeInBytes);
        file.close();
        return true;
    }

    //read a saved depth image. It is not used here, it's just for debugging and make sure the baseline is correct.
    //the saved image can't be opened by a classic image viewer but it can be displayed in an opencv window
    bool readRawImage(cv::Mat& image, const std::string& filename)
    {
      int rows, cols, data, depth, type, channels;
      ifstream file (filename, ios::in|ios::binary);
      if (!file.is_open())
          return false;
      try {
          file.read(reinterpret_cast<char *>(&rows), sizeof(rows));
          file.read(reinterpret_cast<char *>(&cols), sizeof(cols));
          file.read(reinterpret_cast<char *>(&depth), sizeof(depth));
          file.read(reinterpret_cast<char *>(&type), sizeof(type));
          file.read(reinterpret_cast<char *>(&channels), sizeof(channels));
          file.read(reinterpret_cast<char *>(&data), sizeof(data));
          image = cv::Mat(rows, cols, type);
          file.read(reinterpret_cast<char *>(image.data), data);
      } catch (...) {
          file.close();
          return false;
      }

      file.close();
      return true;
    }
    //resize the depthmap to remove black spots all around so we have a more detailed depth image
    void resizeDepthMap()
    {
      crop_min_x = crop_min_x * resize;
      crop_max_x = crop_max_x * resize;
      crop_min_y = crop_min_y * resize;
      crop_max_y = crop_max_y * resize;
    }
    //wirte the extreme values at the calibration folder location
    void writeExtremeValues()
    {
      std::ofstream ofile;
      std::string home = std::getenv("HOME");
      std::string name_file = calibration_folder + "extreme_values.txt";
      ofile.open(name_file, std::ios::app);
      ofile << crop_min_x << std::endl;
      ofile << crop_max_x << std::endl;
      ofile << crop_min_y << std::endl;
      ofile << crop_max_y << std::endl;
      ofile << crop_min_z << std::endl;
      ofile << crop_max_z << std::endl;
      ofile.close();
    }
    //write the linear transform params in the calibration folder
    void writeCalibrationParams()
    {
      std::string name_file = calibration_folder + "params.txt";
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

    //read the linear transform params
    void readParamsDM()
    {
      std::string line;
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
    //read extreme values
    bool readCalibrationFile()
    {
      bool calib = false;
      std::string line;
      std::string name_file = calibration_folder + "extreme_values.txt";
      std::ifstream icpfile(name_file);
      if(icpfile.is_open())
      {
        calib = true;
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
      return calib;
    }
}; 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_fuse");
  FusionPcl fpcl;
  ros::spin();
  return 0;
}
