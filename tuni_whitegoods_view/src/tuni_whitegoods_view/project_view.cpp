#include "tuni_whitegoods_view/project_view.h"

#include <sensor_msgs/image_encodings.h>

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

Projector::Projector(ros::NodeHandle *nh) {

  if (!ros::param::get("shiftX", shift)) {
      shift = 0; // Default value
      ROS_WARN("Parameter 'shiftX' not found, using default value 0.");
  }

  ros::param::get("projector_resolution", projector_resolution);

  ros::param::get("border_homography", border_homography_array);
  ros::param::get("button_homography", button_homography_array);
  border_homography =  cv::Matx33d(border_homography_array.data());
  button_homography =  cv::Matx33d(button_homography_array.data());
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
          ROS_INFO("%f", border_homography(i, j));
      }
  }
  ROS_INFO("ProjectorView running");
}

Projector::~Projector() { cv::destroyWindow(OPENCV_WINDOW); }

void Projector::init() {
  cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
  cv::moveWindow(OPENCV_WINDOW, shift, 0);
  sum_img = cv::Mat::zeros(projector_resolution[1], projector_resolution[0], CV_8UC3); // Black image
  button_img = cv::Mat::zeros(projector_resolution[1], projector_resolution[0], CV_8UC3);
  border_img = cv::Mat::zeros(projector_resolution[1], projector_resolution[0], CV_8UC3); 
  //cv::setWindowProperty(OPENCV_WINDOW, cv::WND_PROP_FULLSCREEN,
  //                      cv::WINDOW_FULLSCREEN);
  try {
      cv::imshow(OPENCV_WINDOW, sum_img);
      cv::waitKey(100);
  } catch (const cv::Exception &e) {
      ROS_INFO("OpenCV error: %s", e.what());
  }
}

void Projector::updateButtons(std::vector<std::shared_ptr<Button>> buttons) {
  button_img = cv::Mat::zeros(projector_resolution[1], projector_resolution[0], CV_8UC3);
  for (auto &button: buttons)
  {
    cv::Mat img_transformed(projector_resolution[1], projector_resolution[0], CV_8UC3, cv::Scalar(0, 0, 0));
    cv::warpPerspective(button->draw(), img_transformed, button_homography, sum_img.size());
    button_img = button_img + img_transformed;
  }
  sum_img = border_img + button_img;
  cv::imshow(OPENCV_WINDOW, sum_img);
  cv::waitKey(100);
}

void Projector::updateBorders(std::vector<std::shared_ptr<StaticBorder>> borders) {
  border_img = cv::Mat::zeros(projector_resolution[1], projector_resolution[0], CV_8UC3);
  for (auto &border: borders)
  {
    cv::Mat img_transformed(projector_resolution[1], projector_resolution[0], CV_8UC3, cv::Scalar(0, 0, 0));
    cv::warpPerspective(border->drawBorder(), img_transformed, border_homography, sum_img.size());
    border_img = border_img + img_transformed;
  }
  sum_img = border_img + button_img;
  cv::imshow(OPENCV_WINDOW, sum_img);
  cv::waitKey(100);
}

void Projector::updateHands(std::vector<std::shared_ptr<Hand>> hands) {}

