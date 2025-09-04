/*
#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <actionlib/client/simple_action_client.h>
#include <imgui.h>
#include <integration/BookOperatorStaticBorderAction.h>
#include <integration/BookOperatorStaticBorderGoal.h>
#include <integration/BookRobotStaticBorderAction.h>
#include <integration/BookRobotStaticBorderGoal.h>
#include <integration/ListStaticBordersStatus.h>
#include <integration/ReleaseOperatorStaticBorderAction.h>
#include <integration/ReleaseOperatorStaticBorderGoal.h>
#include <integration/ReleaseRobotStaticBorderAction.h>
#include <integration/ReleaseRobotStaticBorderGoal.h>
#include <integration/SetLayoutStaticBordersAction.h>
#include <integration/SetLayoutStaticBordersGoal.h>
#include <integration/SetSafetyBorderProjectionAction.h>
#include <integration/SetSafetyBorderProjectionGoal.h>
#include <integration/SetVirtualButtonsProjectionAction.h>
#include <integration/SetVirtualButtonsProjectionGoal.h>
#include <std_msgs/Int32.h>

#include <cstdlib>
#include <queue>
#include <thread>

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "tuni_whitegoods_controller/projector_interface_controller.h"
#include "tuni_whitegoods_msgs/DynamicArea.h"
#include "tuni_whitegoods_msgs/HandsState.h"
#include "tuni_whitegoods_projector_interface/display_area.h"
#include "tuni_whitegoods_view/view.h"

class GUI {
 private:
  ros::NodeHandle* nh_;
  int row_layout, column_layout;
  bool scan;
  bool tf;
  ros::ServiceClient client_detection;
  std::queue<integration::SetSafetyBorderProjectionGoal> goalQueue;
  std::queue<integration::SetVirtualButtonsProjectionGoal> buttonQueue;

  std::vector<int> projector_resolution;

  std::shared_ptr<ProjectorInterfaceController> controller_;

  ros::Subscriber hand_detection_sub;
  ros::Subscriber table_detection_sub;

  ros::Publisher tf_pub;
  ros::Publisher smart_interface_pub;
  ros::Publisher threshold_pub;
  ros::Publisher non_zero_threshold_pub;
  ros::Publisher noise_recuction_pub;

  ros::Publisher pub_max_width;
  ros::Publisher pub_max_height;
  ros::Publisher pub_angle;
  ros::Publisher pub_center;
  ros::Publisher pub_threshold_detection_table;

  void initializeGLFWandOpenGL();
  void initializeImGui(GLFWwindow* window);
  void update_gui();
  void cleanupImGui();
  GLuint cvMatToTexture(const cv::Mat& mat);
  GLFWwindow* window;
  void show_projected_image();
  void show_layer_manager();
  void show_element_creator();
  void show_debug_borders();
  void show_debug_buttons();
  void show_debug_hands();
  void show_debug_object_detection();
  void show_projector_manager();
  void show_moving_table();
  void show_node_starter();
  void launchTfNode();

  actionlib::SimpleActionClient<integration::SetSafetyBorderProjectionAction>
      client_border;
  actionlib::SimpleActionClient<integration::SetLayoutStaticBordersAction> ac;

  actionlib::SimpleActionClient<integration::BookRobotStaticBorderAction>
      client_book_border_robot;
  actionlib::SimpleActionClient<integration::ReleaseRobotStaticBorderAction>
      client_release_border_robot;
  actionlib::SimpleActionClient<integration::BookOperatorStaticBorderAction>
      client_book_border_human;
  actionlib::SimpleActionClient<integration::ReleaseOperatorStaticBorderAction>
      client_release_border_human;
  actionlib::SimpleActionClient<integration::SetVirtualButtonsProjectionAction>
      project_client;

  int hand_detection_counter;
  ros::Time last_msg_time_;
  float interval_in_seconds_;

  int table_detection_counter;
  ros::Time last_table_msg_time_;
  float table_interval_in_seconds_;

 public:
  GUI(ros::NodeHandle* nh,
      std::shared_ptr<ProjectorInterfaceController> controller);
  ~GUI();
  void handDetectionCallback(
      const tuni_whitegoods_msgs::HandsState::ConstPtr& msg);
  void tableDetectionCallback(
      const tuni_whitegoods_msgs::DynamicArea::ConstPtr& msg);
  void update_imgui();
};

#endif
*/