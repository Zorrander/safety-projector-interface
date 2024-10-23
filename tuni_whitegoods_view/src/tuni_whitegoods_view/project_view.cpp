#include "tuni_whitegoods_view/project_view.h"

#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Quaternion.h>

#include "tf2/utils.h"

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

Projector::Projector(ros::NodeHandle *nh) {
  if (!ros::param::get("shiftX", shift)) {
    shift = 0;  // Default value
    ROS_WARN("Parameter 'shiftX' not found, using default value 0.");
  }

  ROS_INFO("ProjectorView running");
}

Projector::~Projector() { cv::destroyWindow(OPENCV_WINDOW); }

void Projector::init(std::vector<std::shared_ptr<DisplayArea>> zones) {
  ros::param::get("projector_resolution", projector_resolution);

  ros::param::get("/border_homography", border_homography_array);
  ros::param::get("/button_homography", button_homography_array);
  border_homography = cv::Matx33d(border_homography_array.data());
  button_homography = cv::Matx33d(button_homography_array.data());
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ROS_INFO("%f", border_homography(i, j));
    }
  }

  cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
  cv::moveWindow(OPENCV_WINDOW, shift, 0);
  cv::setWindowProperty(OPENCV_WINDOW, cv::WND_PROP_FULLSCREEN,
                        cv::WINDOW_FULLSCREEN);
  sum_img = cv::Mat::zeros(projector_resolution[1], projector_resolution[0],
                           CV_8UC3);  // Black image
  button_img =
      cv::Mat::zeros(projector_resolution[1], projector_resolution[0], CV_8UC3);
  border_img =
      cv::Mat::zeros(projector_resolution[1], projector_resolution[0], CV_8UC3);

  for (auto &zone : zones) {
    if (!(zone->name == "projector" || zone->name == "camera")) {
      const cv::Scalar color(255, 255, 255);
      const int thickness = 10;
      for (size_t i = 0; i < zone->projector_frame_area.size(); ++i) {
        ROS_INFO_STREAM("Point " << i << ": ("
                                 << zone->projector_frame_area[i].x << ", "
                                 << zone->projector_frame_area[i].y << ")");
      }
      cv::polylines(sum_img, zone->projector_frame_area, true, color, thickness,
                    cv::LINE_8);

      const int fontFace = cv::FONT_HERSHEY_SIMPLEX;
      const double fontScale = 1.0;  // Adjust for text size
      const int textThickness = 2;
      const cv::Scalar textColor(255, 255, 255);  // Green color in BGR

      cv::putText(sum_img, zone->name, zone->projector_frame_area[0], fontFace,
                  fontScale, textColor, textThickness);
    }
  }

  try {
    cv::imshow(OPENCV_WINDOW, sum_img);
    cv::waitKey(100);
  } catch (const cv::Exception &e) {
    ROS_INFO("OpenCV error: %s", e.what());
  }
}

void Projector::updateButtons(
    const std::vector<std::shared_ptr<Button>> &buttons) {
  button_img =
      cv::Mat::zeros(projector_resolution[1], projector_resolution[0], CV_8UC3);
  for (auto &button : buttons) {
    cv::Mat img_transformed(projector_resolution[1], projector_resolution[0],
                            CV_8UC3, cv::Scalar(0, 0, 0));
    cv::warpPerspective(button->draw(), img_transformed, button_homography,
                        sum_img.size());
    button_img = button_img + img_transformed;
  }
  sum_img = border_img + button_img;
  cv::imshow(OPENCV_WINDOW, sum_img);
  cv::waitKey(100);
}

void Projector::updateBorders(
    const std::vector<std::shared_ptr<StaticBorder>> &borders) {
  border_img =
      cv::Mat::zeros(projector_resolution[1], projector_resolution[0], CV_8UC3);
  for (auto &border : borders) {
    cv::Mat img_transformed(projector_resolution[1], projector_resolution[0],
                            CV_8UC3, cv::Scalar(0, 0, 0));
    cv::warpPerspective(border->drawBorder(), img_transformed,
                        border_homography, sum_img.size());
    border_img = border_img + img_transformed;
  }
  sum_img = border_img + button_img;
  cv::imshow(OPENCV_WINDOW, sum_img);
  cv::waitKey(100);
}

void Projector::updateHands(const std::vector<std::shared_ptr<Hand>> &hands) {}

void Projector::updateDisplayAreas(
    const std::vector<std::shared_ptr<DisplayArea>> &zones) {
  ROS_INFO("No view to update");
}
