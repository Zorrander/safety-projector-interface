#include <ros/ros.h>
// PCL specific includes
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <ros/header.h>

static const std::string OPENCV_WINDOW = "rgb";

class CalibExtrinsicCamera
{
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    cv_bridge::CvImagePtr cv_ptr;
    image_transport::Subscriber rgb_sub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::Buffer tfBuffer_d;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_d;
    geometry_msgs::TransformStamped transformStamped_rgb_depth;
    geometry_msgs::TransformStamped transformStamped_base_rgb;
    Eigen::Matrix4d robot_frame;
    cv::Mat rgb_img;
    cv::Mat thre;
    bool record;
    bool init;
    int low_H;
    int low_S;
    int low_V;
    int high_H;
    int high_S;
    int high_V;
    bool tf_in;
    bool tf_rgb;
  public:
    CalibExtrinsicCamera(ros::NodeHandle *nh):
    nh_(*nh),
    it_(nh_)
    {
      rgb_sub = it_.subscribe("/rgb/image_raw", 1,&CalibExtrinsicCamera::rgbCallback, this);
      tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);
      tfListener_d = std::make_shared<tf2_ros::TransformListener>(tfBuffer_d);
      record = true;
      init = false;
      low_H = 170;
      low_S = 149;
      low_V = 147;
      high_H = 178;
      high_S = 243;
      high_V = 224;
      tf_in = false;
      tf_rgb = false;
    }

    ~CalibExtrinsicCamera()
    {
      //cv::destroyWindow(OPENCV_WINDOW);
      cv::destroyWindow(OPENCV_WINDOW);
    }

    void rgbCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      thre = cv::Mat(1280, 720, CV_8U,cv::Scalar(std::numeric_limits<float>::min()));;
      cv::Mat thr;
      listenTransform();
      if(record)
      {
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          rgb_img = cv_ptr->image.clone();
          //cv::imwrite("/home/altair/odin/src/calibration/scripts/sample.jpg", rgb_img);
          init = true;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //record = false;
        
      }
    }

    void listenTransform()
    {
      bool error = false;
      if(!tf_in)
      {
        try
        {
          tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);
          
          transformStamped_base_rgb = tfBuffer.lookupTransform("base", "rgb_camera_link",ros::Time(30.0));
          //tfListener_d waitForTransform("/depth_camera_link","/rgb_camera_link", ros::Time(), ros::Duration(5.0));
          //transformStamped_rgb_depth = tfBuffer_d.lookupTransform("rgb_camera_link", "depth_camera_link",ros::Time(35.0));
          
        } 
        catch (tf2::TransformException &ex) 
        {
          ROS_WARN("%s", ex.what());
          error = true;
          ros::Duration(1.0).sleep();
        }
        if(error == false)
        {
          std::cout<<"got transform Base\n";
          tf_in = true;
          tf2::Quaternion qr(transformStamped_base_rgb.transform.rotation.x,transformStamped_base_rgb.transform.rotation.y,transformStamped_base_rgb.transform.rotation.z,transformStamped_base_rgb.transform.rotation.w);
          double roll, pitch, yaw;
          tf2::Matrix3x3 m(qr);
          m.getRPY(roll,pitch,yaw);
          Eigen::Isometry3d mat = tf2::transformToEigen(transformStamped_rgb_depth);
          robot_frame = mat.matrix();
          

          std::cout<<"position : "<<transformStamped_base_rgb.transform.translation.x<<" "<<transformStamped_base_rgb.transform.translation.y<<" "
          <<transformStamped_base_rgb.transform.translation.z<<" orientation : "<<transformStamped_base_rgb.transform.rotation.x<<" "<<transformStamped_base_rgb.transform.rotation.y
          <<" "<<transformStamped_base_rgb.transform.rotation.z<<" "<<transformStamped_base_rgb.transform.rotation.w<<"\n";
          std::cout<< "rpy : "<<roll<<" "<<pitch<<" "<<yaw<<"\n";
          double r=-0.18, p=0.12, y=0.6;  // Rotate the previous pose by 180* about X
          tf2::Quaternion q_rot, q_new;
          q_rot.setRPY(r, p, y);
          q_new = q_rot*qr;
          std::cout<<" NEW orientation : "<<q_new.getX()<<" "<<q_new.getY()<<" "<<q_new.getZ()<<" "<<q_new.getW()<<"\n";

          //std::cout<<"tf2 Transform : "<<t_rgb.
          /*std::cout<<"position : "<<transformStamped_rgb_depth.transform.translation.x<<" "<<transformStamped_rgb_depth.transform.translation.y<<" "
          <<transformStamped_rgb_depth.transform.translation.z<<" orientation : "<<transformStamped_rgb_depth.transform.rotation.x<<" "<<transformStamped_rgb_depth.transform.rotation.y
          <<" "<<transformStamped_rgb_depth.transform.rotation.z<<" "<<transformStamped_rgb_depth.transform.rotation.w<<"\n";
          std::cout<<"position : "<<p.position.x<<" "<<p.position.y<<" "<<p.position.z<<" orientation  : "<<p.orientation.x<<" "<<p.orientation.y<<" "<<p.orientation.z<<" "<<p.orientation.w<<"\n";*/
        }      
      }
    }

    void listenTransformCamera()
    {
      bool error = false;
      if(!tf_rgb)
      {
        try
        {
          tfListener_d = std::make_shared<tf2_ros::TransformListener>(tfBuffer_d);
          
          //transformStamped_base_rgb = tfBuffer.lookupTransform("base", "rgb_camera_link",ros::Time(30.0));
          //tfListener_d waitForTransform("/depth_camera_link","/rgb_camera_link", ros::Time(), ros::Duration(5.0));
          transformStamped_rgb_depth = tfBuffer_d.lookupTransform("depth_camera_link", "rgb_camera_link",ros::Time(35.0));
          
        } 
        catch (tf2::TransformException &ex) 
        {
          ROS_WARN("%s", ex.what());
          error = true;
          ros::Duration(1.0).sleep();
        }
        if(error == false)
        {
          std::cout<<"got transform Camera\n";
          tf2::Transform t_rgb;
          tf2::Transform t_depth;
          tf2::Transform t_mul;
          tf2::fromMsg(transformStamped_base_rgb.transform,t_rgb);
          tf2::fromMsg(transformStamped_rgb_depth.transform,t_depth);
          tf2::Transform base_to_depth = t_rgb * t_depth;
          geometry_msgs::Pose p;
          //tf2::toMsg(base_to_depth,p);
          p.position.x = transformStamped_base_rgb.transform.translation.x + transformStamped_rgb_depth.transform.translation.x;
          p.position.y = transformStamped_base_rgb.transform.translation.y + transformStamped_rgb_depth.transform.translation.y;
          p.position.z = transformStamped_base_rgb.transform.translation.z + transformStamped_rgb_depth.transform.translation.z;
          //p.orientation.x = 0.742978 + transformStamped_rgb_depth.transform.rotation.x;
          //p.orientation.y = -0.665625 + transformStamped_rgb_depth.transform.rotation.y;
          //p.orientation.z = 0.060038 + transformStamped_rgb_depth.transform.rotation.z;
          //p.orientation.w = -0.036349 + transformStamped_rgb_depth.transform.rotation.w;
          t_mul.mult(t_rgb,t_depth);
          tf2::Quaternion qr(t_mul.getRotation());
          p.orientation.x = qr.getX();
          p.orientation.y = qr.getY();
          p.orientation.z = qr.getZ();
          p.orientation.w = qr.getW();

          Eigen::Isometry3d mat = tf2::transformToEigen(transformStamped_rgb_depth);
          robot_frame = mat.matrix();
          tf_rgb = true;

          /*std::cout<<"position : "<<transformStamped_base_rgb.transform.translation.x<<" "<<transformStamped_base_rgb.transform.translation.y<<" "
          <<transformStamped_base_rgb.transform.translation.z<<" orientation : "<<transformStamped_base_rgb.transform.rotation.x<<" "<<transformStamped_base_rgb.transform.rotation.y
          <<" "<<transformStamped_base_rgb.transform.rotation.z<<" "<<transformStamped_base_rgb.transform.rotation.w<<"\n";
          //std::cout<< "rpy : "<<roll<<" "<<pitch<<" "<<yaw<<"\n";
          //std::cout<<"tf2 Transform : "<<t_rgb.
          std::cout<<"position : "<<transformStamped_rgb_depth.transform.translation.x<<" "<<transformStamped_rgb_depth.transform.translation.y<<" "
          <<transformStamped_rgb_depth.transform.translation.z<<" orientation : "<<transformStamped_rgb_depth.transform.rotation.x<<" "<<transformStamped_rgb_depth.transform.rotation.y
          <<" "<<transformStamped_rgb_depth.transform.rotation.z<<" "<<transformStamped_rgb_depth.transform.rotation.w<<"\n";
          std::cout<<"position : "<<p.position.x<<" "<<p.position.y<<" "<<p.position.z<<" orientation  : "<<p.orientation.x<<" "<<p.orientation.y<<" "<<p.orientation.z<<" "<<p.orientation.w<<"\n";*/
        }      
      }
    }

    cv::Mat filterHSVImage()
    {
      cv::Mat hsv_img;
      cv::Mat threshold_img;
      cv::cvtColor(rgb_img, hsv_img, cv::COLOR_BGR2HSV);
      inRange(hsv_img, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), threshold_img);
  
      return threshold_img;
    }

    cv::Point getCenterPoint(cv::Mat img)
    {
      cv::Mat tmp;
      cv::threshold(img, tmp, 100,255,cv::THRESH_BINARY );    
      // find moments of the image
      cv::Moments m = cv::moments(tmp,true);
      cv::Point p(m.m10/m.m00, m.m01/m.m00);

      return p;
    }

    cv::Mat getRGBImage()
    {
      return rgb_img;
    }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "detect_interaction");
  bool continues = true;
  cv::namedWindow(OPENCV_WINDOW,cv::WINDOW_NORMAL);
  ros::NodeHandle nh_calib;
  boost::shared_ptr<ros::AsyncSpinner> g_spinner;
  ros::CallbackQueue queue;
  nh_calib.setCallbackQueue(&queue);
  g_spinner.reset(new ros::AsyncSpinner(0, &queue));
  CalibExtrinsicCamera cal(&nh_calib);
  g_spinner->start();
  queue.callAvailable(); 
  ros::Duration(1.0).sleep();
  while(ros::ok() && continues)
  {
    cal.listenTransform();
    //cal.listenTransformCamera();
    /*cv::Mat thre;
    cv::Mat rgb_im;
    
    rgb_im = cal.getRGBImage();
    thre = cal.filterHSVImage();
    cv::Point p = cal.getCenterPoint(thre);
    // show the image with a point mark at the centroid
    cv::circle(rgb_im, p, 5, cv::Scalar(255,255,255), -1);
    cv::imshow(OPENCV_WINDOW, rgb_im);
    int c = cv::waitKey(0);
    if(c == 113)
    {
      continues = false;
    }
    else
    {
      queue.callAvailable(); 
    }*/
    queue.callAvailable(); 
  }
  g_spinner->stop();

  return 0;
}