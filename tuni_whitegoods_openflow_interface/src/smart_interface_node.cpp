#include <ros/ros.h>

#include "projector_interface/book_operator_static_border_server.h"
#include "projector_interface/book_robot_static_border_server.h"
#include "projector_interface/button_color_server.h"
#include "projector_interface/button_projection_server.h"
#include "projector_interface/instruction_projection_server.h"
#include "projector_interface/release_operator_static_border_server.h"
#include "projector_interface/release_robot_static_border_server.h"
#include "projector_interface/safety_border_server.h"
#include "projector_interface/static_border_server.h"
#include "projector_interface/unset_projection_server.h"
#include "projector_interface/user_interface_server.h"
#include "tuni_whitegoods_controller/projector_interface_controller.h"

/**
 * @mainpage ODIN %Projector Interface Documentation
 *
 * Welcome to the documentation for **ODIN %Projector Interface**!
 *
 * ## Getting Started
 *
 * To get started with **ODIN %Projector Interface**, follow these steps:
 *
 * 1. Clone the repository.
 * 2. Install the necessary dependencies.
 * 3. Build the project using `catkin_make`.
 *
 */

int main(int argc, char** argv) {
  ros::init(argc, argv, "smart_interface_node");

  ros::NodeHandle nh;

  std::string button_projection_server_name, button_color_server_name,
      user_interface_server_name, unset_projection_server_name,
      instruction_projection_server_name, static_border_server_name,
      book_robot_static_border_server_name,
      release_robot_static_border_server_name,
      book_operator_static_border_server_name,
      release_operator_static_border_server_name, safety_border_server_name;

  ros::param::get("button_projection_server_name",
                  button_projection_server_name);
  ros::param::get("button_color_server_name", button_color_server_name);
  ros::param::get("user_interface_server_name", user_interface_server_name);
  ros::param::get("unset_projection_server_name", unset_projection_server_name);
  ros::param::get("instruction_projection_server_name",
                  instruction_projection_server_name);
  static_border_server_name =
      "execution/projector_interface/integration/actions/"
      "set_layout_static_borders";

  ros::param::get("book_robot_static_border_server",
                  book_robot_static_border_server_name);
  ros::param::get("release_robot_static_border_server",
                  release_robot_static_border_server_name);
  ros::param::get("book_operator_static_border_server",
                  book_operator_static_border_server_name);
  ros::param::get("release_operator_static_border_server",
                  release_operator_static_border_server_name);
  ros::param::get("safety_border_server", safety_border_server_name);

  // Instantiate th core of the software
  std::shared_ptr<ProjectorInterfaceController> controller =
      std::make_shared<ProjectorInterfaceController>(&nh);

  // Create an action server object and spin ROS
  UserInterfaceServer srv4(&nh, user_interface_server_name, controller);

  ButtonProjectionServer srv2(&nh, button_projection_server_name, controller);
  ButtonColorServer srv3(&nh, button_color_server_name, controller);

  UnsetProjectionServer srv5(&nh, unset_projection_server_name, controller);
  InstructionProjectionServer srv6(&nh, instruction_projection_server_name,
                                   controller);

  BookRobotStaticBorderServer book_robot_static_border_server(
      &nh, book_robot_static_border_server_name, controller);
  ReleaseRobotStaticBorderServer release_robot_static_border_server(
      &nh, release_robot_static_border_server_name, controller);
  BookOperatorStaticBorderServer book_operator_static_border_server(
      &nh, book_operator_static_border_server_name, controller);
  ReleaseOperatorStaticBorderServer release_operator_static_border_server(
      &nh, release_operator_static_border_server_name, controller);

  StaticBorderServer static_border_server(&nh, static_border_server_name,
                                          controller);
  SafetyBorderServer safety_border_server(&nh, safety_border_server_name,
                                          controller);

  ros::spin();

  return 0;
}