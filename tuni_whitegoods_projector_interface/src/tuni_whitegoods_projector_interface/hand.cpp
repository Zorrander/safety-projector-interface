#include "tuni_whitegoods_projector_interface/hand.h"

Hand::Hand(std::string name) : name(name) {}

void Hand::set_position(geometry_msgs::Point pixel,
                        geometry_msgs::Point projected,
                        geometry_msgs::Point robot) {
  pixel_position = pixel;
  projected_position = projected;
  robot_frame_position = robot;
}
