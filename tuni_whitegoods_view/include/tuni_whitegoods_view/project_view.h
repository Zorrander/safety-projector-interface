#ifndef Projector_H
#define Projector_H

#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Float64MultiArray.h>
#include <tuni_whitegoods_msgs/Projection.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "tuni_whitegoods_msgs/DynamicArea.h"
#include "tuni_whitegoods_msgs/HandsState.h"
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
  ros::Subscriber table_detection_sub;
  float top_left_moving_table_x, top_left_moving_table_y,
      top_right_moving_table_x, top_right_moving_table_y,
      bottom_left_moving_table_x, bottom_left_moving_table_y,
      bottom_right_moving_table_x, bottom_right_moving_table_y;

  std::vector<std::shared_ptr<DisplayArea>> display_areas;   

  int height_moving_table, width_moving_table;
  

 public:
  Projector(ros::NodeHandle* nh, int id);
  ~Projector();

  void init(std::vector<std::shared_ptr<DisplayArea>> zones) override;
  void moveWindow() override;
  void updateButtons(const std::vector<std::shared_ptr<Button>>& buttons,
                     std::shared_ptr<cv::Mat> layer) override;
  void updateBorders(const std::vector<std::shared_ptr<StaticBorder>>& borders,
                     std::shared_ptr<cv::Mat> layer) override;
  void updateHands(const std::vector<std::shared_ptr<Hand>>& hands) override;
  void updateDisplayAreas(
      const std::vector<std::shared_ptr<DisplayArea>>& zones) override;
  void project_image();

  bool containsArea(const std::shared_ptr<DisplayArea> zone);

  void tableDetectionCallback(
      const tuni_whitegoods_msgs::DynamicArea::ConstPtr& msg);

  int id_;


};

#endif