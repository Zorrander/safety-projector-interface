#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>


class ObjectDetector
{
public:
    ObjectDetector(ros::NodeHandle* nh)
    {
		sub_scene = nh_->subscribe("odin/visualization/scene_detection", 1, &DepthInterface::depthSceneCallback, this);

    }

private:


	  void StaticBorderManager::depthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg){
		     cv_bridge::CvImagePtr cv_bridge_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
		     cv_depth = cv_bridge_depth->image.clone();
		        
		     for(const auto& sb : borders)
		     {  
		        roi_rect = cv::Rect(sb->top_left_cam_point, sb->bottom_right_cam_point);

		        cv::Mat roi_depthmap = cv_depth(roi_rect);
		        // Apply the depth range threshold
		        cv::Mat mask;
		        cv::Scalar min_depth = cv::Scalar(1250); // TO BE ADJUSTED
		        cv::Scalar max_depth = cv::Scalar(1300); // TO BE ADJUSTED
		        cv::inRange(roi_depthmap, min_depth, max_depth, mask);

		        // Find contours in the mask
		        std::vector<std::vector<cv::Point>> contours;
		        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		        // Find center
		        /*
		        for c in cnts:
		          # compute the center of the contour
		          M = cv2.moments(c)
		          cX = int(M["m10"] / M["m00"])
		          cY = int(M["m01"] / M["m00"])
		        */ 
		        

		        // Check if any contour is found
		        if (contours.size() > 0){
		           sb->changeThickness(-1);
		           sb->occupied = true;       
		        } else {
		           sb->occupied = false;
		        }
		     }
		     
		  }
	



    ros::NodeHandle* nh_;
    

    cv::Mat cv_depth;
    ros::Subscriber sub_scene;

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detection_node");

  ros::NodeHandle nh;
  DepthInterface ObjectDetector(&nh);
  ROS_INFO("object_detection_node running.");
  ros::waitForShutdown();

  return 0;
}




