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
  ros::param::get("projector_resolution", projector_resolution);
  layers["background"] = std::make_shared<cv::Mat>(cv::Mat::zeros(
      projector_resolution[1], projector_resolution[0], CV_8UC3));
  cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
  cv::moveWindow(OPENCV_WINDOW, shift, 0);
  cv::setWindowProperty(OPENCV_WINDOW, cv::WND_PROP_FULLSCREEN,
                        cv::WINDOW_FULLSCREEN);
  ROS_INFO("ProjectorView running");
}

Projector::~Projector() { cv::destroyWindow(OPENCV_WINDOW); }

void Projector::init(std::vector<std::shared_ptr<DisplayArea>> zones) {
  for (auto &zone : zones) {
    if (!(zone->name == "projector" || zone->name == "camera")) {
      cv::Point tl(zone->projector_frame_area[0].x,
                   zone->projector_frame_area[0].y);
      cv::Point tr(zone->projector_frame_area[1].x,
                   zone->projector_frame_area[1].y);
      cv::Point br(zone->projector_frame_area[2].x,
                   zone->projector_frame_area[2].y);
      cv::Point bl(zone->projector_frame_area[3].x,
                   zone->projector_frame_area[3].y);

      std::vector<cv::Point> rectanglePoints = {tl, tr, br, bl};

      layers[zone->name] = std::make_shared<cv::Mat>(cv::Mat::zeros(
          projector_resolution[1], projector_resolution[0], CV_8UC3));

      cv::polylines(*layers[zone->name], rectanglePoints, true, cv::Scalar(255),
                    10, cv::LINE_8);
    }
  }
  project_image();
}

void Projector::updateButtons(
    const std::vector<std::shared_ptr<Button>> &buttons,
    std::shared_ptr<cv::Mat> layer) {
  for (auto &button : buttons) {
    cv::circle(*layer, button->center_projected_point, button->radius,
               button->btn_color, -1);
  }
}

void Projector::updateBorders(
    const std::vector<std::shared_ptr<StaticBorder>> &borders,
    std::shared_ptr<cv::Mat> layer) {
  for (auto &border : borders) {
    /*
    cv::rectangle(
        *layer, border->top_left_proj_point, border->bottom_right_proj_point,
        cv::Scalar(border->border_color.b * 255, border->border_color.g * 255,
                   border->border_color.r * 255),
        border->thickness * 2, cv::LINE_8);
    */
  }
}

void Projector::updateHands(const std::vector<std::shared_ptr<Hand>> &hands) {}

void Projector::updateDisplayAreas(
    const std::vector<std::shared_ptr<DisplayArea>> &zones) {
  for (auto &zone : zones) {
    if (!(zone->name == "projector" || zone->name == "camera")) {
      cv::Point tl(zone->projector_frame_area[0].x,
                   zone->projector_frame_area[0].y);
      cv::Point tr(zone->projector_frame_area[1].x,
                   zone->projector_frame_area[1].y);
      cv::Point br(zone->projector_frame_area[2].x,
                   zone->projector_frame_area[2].y);
      cv::Point bl(zone->projector_frame_area[3].x,
                   zone->projector_frame_area[3].y);

      std::vector<cv::Point> rectanglePoints = {tl, tr, br, bl};

      const cv::Scalar color(255, 255, 255);
      const int thickness = 10;

      layers[zone->name] = std::make_shared<cv::Mat>(cv::Mat::zeros(
          projector_resolution[1], projector_resolution[0], CV_8UC3));

      cv::polylines(*layers[zone->name], rectanglePoints, true, cv::Scalar(255),
                    10, cv::LINE_8);

      std::vector<std::shared_ptr<Button>> buttons;
      zone->fetchButtons(buttons);
      updateButtons(buttons, layers[zone->name]);

      std::vector<std::shared_ptr<StaticBorder>> borders;
      zone->fetchBorders(borders);
      updateBorders(borders, layers[zone->name]);
    }
  }
  project_image();
}

void Projector::project_image() {
  cv::Mat combined = layers["background"]->clone();

  for (auto it = layers.begin(); it != layers.end(); ++it) {
    const std::string &name = it->first;
    std::shared_ptr<cv::Mat> &layer = it->second;
    cv::bitwise_or(combined, *layer, combined);
  }
  cv::imshow(OPENCV_WINDOW, combined);
  cv::waitKey(100);
}
