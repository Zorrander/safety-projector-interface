#ifndef Button_H
#define Button_H

#include <ros/ros.h>
#include <string>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Button {
private:
	std::string name;
	std::string text;	
  

  cv::Mat btn_img;
  bool left_hand_press, right_hand_press, button_pressed;


public:
  cv::Scalar btn_color, txt_color;
  float radius;
  Button(std::string name, std::string text, 
         std_msgs::ColorRGBA button_color, std_msgs::ColorRGBA text_color, 
         geometry_msgs::Pose center, float radius);
  cv::Mat draw();
  geometry_msgs::Pose center;
  cv::Point center_cam_point;
  void resetInteractions();
  bool checkForInteractions(std::string name, cv::Point hand_position);
  void set_button_color(std_msgs::ColorRGBA button_color);
  void set_text_color(std_msgs::ColorRGBA text_color);
  std::string get_name();
};

#endif