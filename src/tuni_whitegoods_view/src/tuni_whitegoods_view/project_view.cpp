#include "tuni_whitegoods_view/project_view.h"

#include <sensor_msgs/image_encodings.h>

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

Projector::Projector() {

  if (!ros::param::get("shiftX", shift)) {
      shift = 0; // Default value
      ROS_WARN("Parameter 'shiftX' not found, using default value 0.");
  }


  border_hom = cv::Matx33d(1.70497912e+00,  4.42399276e-01, -7.37314713e+02,
                    5.19667675e-02 , 2.20520788e+00, -4.49266091e+02,
                    5.66463639e-05  ,5.47137996e-04,  1.00000000e+00);

  button_hom = cv::Matx33d(2.96464922e+00, -1.07291215e+00, -6.60992620e+02,
                           1.39999849e+00,  6.79257520e-01, -3.23236961e+02,
                           2.41945642e-03, -2.21125947e-03,  1.00000000e+00);

  ROS_INFO("ProjectorView running");
}

Projector::~Projector() { cv::destroyWindow(OPENCV_WINDOW); }

void Projector::init() {
  cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
  cv::moveWindow(OPENCV_WINDOW, shift, 0);
  sum_img = cv::Mat::zeros(1080, 1920, CV_8UC3); // Black image
  button_img = cv::Mat::zeros(1080, 1920, CV_8UC3);
  border_img = cv::Mat::zeros(1080, 1920, CV_8UC3); 
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
  button_img = cv::Mat::zeros(1080, 1920, CV_8UC3);
  for (auto &button: buttons)
  {
    cv::Mat img_transformed(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::warpPerspective(button->draw(), img_transformed, button_hom, sum_img.size());
    button_img = button_img + img_transformed;
  }
  sum_img = border_img + button_img;
  cv::imshow(OPENCV_WINDOW, sum_img);
  cv::waitKey(100);
}

void Projector::updateBorders(std::vector<std::shared_ptr<StaticBorder>> borders) {
  border_img = cv::Mat::zeros(1080, 1920, CV_8UC3);
  for (auto &border: borders)
  {
    cv::Mat img_transformed(1080, 1920, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::warpPerspective(border->drawBorder(), img_transformed, border_hom, sum_img.size());
    border_img = border_img + img_transformed;
  }
  sum_img = border_img + button_img;
  cv::imshow(OPENCV_WINDOW, sum_img);
  cv::waitKey(100);
}

void Projector::updateHands(std::vector<std::shared_ptr<Hand>> hands) {}

