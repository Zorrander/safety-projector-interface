#include "tuni_whitegoods_projector_interface/button.h"

Button::Button(std::string request_id, std::string name, std::string text, 
               std_msgs::ColorRGBA button_color, std_msgs::ColorRGBA text_color, 
               geometry_msgs::Pose center, float radius) 
			: name(name),
		    text(text),
			  center(center),
			  radius(radius),
        id(request_id)
{
  set_button_color(button_color);
  set_text_color(text_color);
  button_already_pressed = false;
}

cv::Mat Button::draw() {
  btn_img = cv::Mat::zeros(1080, 1920, CV_8UC3);

  cv::circle(btn_img, center_cam_point, radius, btn_color, -1);

  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 0.5;
  int thickness = 2;

  // Calculate the size of the text box
  int baseline = 0;
  cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline);

  // Calculate the bottom-left corner of the text box to center it
  cv::Point text_origin(
      center_cam_point.x - textSize.width / 2,
      center_cam_point.y + textSize.height / 2
  );

  cv::putText(btn_img, text, text_origin, fontFace, fontScale, txt_color, thickness);

  return btn_img;
}

bool Button::isAlreadyPressed(){
  return button_already_pressed;
}

void Button::setAlreadyPressed(bool alreadyPressed){
  button_already_pressed = alreadyPressed;
}

bool Button::checkForInteractions(std::string name,
                                        cv::Point hand_position) {
  bool result = false;
  float distance = cv::norm(hand_position - center_cam_point);
  bool is_crossed = distance < radius*1.2;
  if (name == "left") {
    left_hand_press = is_crossed;
  } else if (name == "right") {
    right_hand_press = is_crossed;
  }

  button_pressed = (left_hand_press || right_hand_press);

  if (button_pressed){
    btn_color = cv::Scalar(0, 0, 255);
    result = true;
  } else {
    btn_color = cv::Scalar(255,0, 0);  }  
  return result;
}

void Button::resetInteractions() {
  button_pressed = false;
  btn_color = cv::Scalar(255, 0, 0);
}

std::string Button::get_name() {
  return name;
}

void Button::set_button_color(std_msgs::ColorRGBA button_color) {
  ros_btn_color = button_color;
  base_btn_color= cv::Scalar(button_color.b * 255, button_color.g * 255, button_color.r * 255, button_color.a * 255);
  btn_color = cv::Scalar(button_color.b * 255, button_color.g * 255, button_color.r * 255, button_color.a * 255);
}

void Button::set_text_color(std_msgs::ColorRGBA text_color) {
  ros_text_color = text_color;
  txt_color = cv::Scalar(text_color.b * 255, text_color.g * 255, text_color.r * 255, text_color.a * 255);
}

std::string Button::getId() { return id; }


            