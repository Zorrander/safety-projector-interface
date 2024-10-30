#ifndef Projector_H
#define Projector_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
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
#include <std_msgs/Float64MultiArray.h>
#include <tuni_whitegoods_msgs/Projection.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "tuni_whitegoods_projector_interface/display_area.h"
#include "tuni_whitegoods_view/view.h"

struct Layer {
  std::shared_ptr<cv::Mat> mat;
  bool visible;
};

class Projector : public View {
 private:
  ros::NodeHandle* nh_;
  cv_bridge::CvImagePtr cv_ptr;
  int shift;
  bool is_moving;
  cv::Mat sum_img, button_img, border_img;
  cv::Mat homography_matrix;
  cv::Matx33d border_homography, button_homography;
  std::vector<double> border_homography_array, button_homography_array;
  std::vector<int> projector_resolution;
  ros::Subscriber transform_callback;
  std::map<std::string, Layer> layers;
  void transformCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  cv::Mat combined;

  ros::ServiceClient client_detection;
  integration::ListStaticBordersStatus srv;

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

  std::vector<std::shared_ptr<DisplayArea>> display_areas;

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

 public:
  Projector(ros::NodeHandle* nh);
  ~Projector();

  void init(std::vector<std::shared_ptr<DisplayArea>> zones) override;
  void updateButtons(const std::vector<std::shared_ptr<Button>>& buttons,
                     std::shared_ptr<cv::Mat> layer) override;
  void updateBorders(const std::vector<std::shared_ptr<StaticBorder>>& borders,
                     std::shared_ptr<cv::Mat> layer) override;
  void updateHands(const std::vector<std::shared_ptr<Hand>>& hands) override;
  void updateDisplayAreas(
      const std::vector<std::shared_ptr<DisplayArea>>& zones) override;
  void project_image();
};

#endif