#include "tuni_whitegoods_view/project_view.h"

#include <cairomm/cairomm.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>
#include <boost/algorithm/clamp.hpp>
#include <cmath>

#include "tf2/utils.h"

using namespace std;

Projector::Projector(ros::NodeHandle *nh, int id) : nh_(nh), id_(id) {
  if (!ros::param::get("shiftX", shift)) {
    shift = 0;  // Default value
  } else {
    shift = shift * id_;
  }
  ros::param::get("projector_resolution", projector_resolution);

  window_name = "workspace";

  layers["background"] = {
      std::make_shared<cv::Mat>(cv::Mat::zeros(
          projector_resolution[1], projector_resolution[0], CV_8UC3)),
      true};

  table_detection_sub =
      nh_->subscribe("/odin/projector_interface/moving_table/transform", 5,
                     &Projector::tableDetectionCallback, this);

  TEXT_FACE = cv::FONT_HERSHEY_DUPLEX;
  TEXT_SCALE_TITLE = 2.0;
  TEXT_SCALE = 1.0;
  TEXT_THICKNESS = 2;
}

Projector::~Projector() { cv::destroyWindow(window_name); }

void Projector::init(std::vector<std::shared_ptr<DisplayArea>> zones) {
  display_areas = zones;
  combined = layers["background"].mat->clone();
  for (auto &zone : zones) {
    if (!(zone->name == "projector" || zone->name == "camera")) {
      layers[zone->name] = {
          std::make_shared<cv::Mat>(cv::Mat::zeros(
              projector_resolution[1], projector_resolution[0], CV_8UC3)),
          true};
    }
  }

  ROS_INFO("Create backend");
  cv::namedWindow(window_name, cv::WINDOW_FULLSCREEN);
  cv::moveWindow(window_name, shift, 0);
  cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN,
                        cv::WINDOW_FULLSCREEN);
  ROS_INFO("projecting");
  project_image();
  ROS_INFO("projected");
}

void Projector::init() {
  combined =
      cv::Mat::zeros(projector_resolution[1], projector_resolution[0], CV_8UC3);

  cv::namedWindow(window_name, cv::WINDOW_FULLSCREEN);
  cv::moveWindow(window_name, shift, 0);
  cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN,
                        cv::WINDOW_FULLSCREEN);
  ROS_INFO("projecting");
  project_image();
  ROS_INFO("projected");
}

void Projector::moveWindow() {
  cv::moveWindow(window_name, shift, 0);

  project_image();
}

/*

Recalibrate

Change relative -> to absolute conversion

Stop filling circle

*/

void Projector::updateButtons(
    const std::vector<std::shared_ptr<Button>> &buttons,
    std::shared_ptr<cv::Mat> layer) {
  int width = layer->cols;
  int height = layer->rows;

  auto surface =
      Cairo::ImageSurface::create(Cairo::Format::FORMAT_RGB24, width, height);
  auto cr = Cairo::Context::create(surface);

  for (auto &button : buttons) {
    // rectangle around that to see what the projection area would be if
    // straight

    /*
    int top_left_straight_table_x, top_left_straight_table_y,
        top_right_straight_table_x, top_right_straight_table_y;

    top_left_straight_table_x =
        button->center_projected_point.x - width_moving_table / 2;
    top_left_straight_table_y =
        button->center_projected_point.y - height_moving_table / 2;
    top_right_straight_table_x =
        button->center_projected_point.x + width_moving_table / 2;
    top_right_straight_table_y =
        button->center_projected_point.y - height_moving_table / 2;

    Eigen::Vector2d point1(top_left_straight_table_x,
                           top_left_straight_table_y);
    Eigen::Vector2d point2(top_right_straight_table_x,
                           top_right_straight_table_y);
    Eigen::Vector2d point3(top_left_moving_table_x, top_left_moving_table_y);
    Eigen::Vector2d point4(top_right_moving_table_x, top_right_moving_table_y);

    // Calculate rotation angle

    Eigen::Vector2d direction1 = point2 - point1;
    Eigen::Vector2d direction2 = point4 - point3;

    Eigen::Vector2d normLine1 = direction1.normalized();
    Eigen::Vector2d normLine2 = direction2.normalized();

    double cosTheta = normLine1.dot(normLine2);
    cosTheta = boost::algorithm::clamp(cosTheta, -1.0, 1.0);

    double angleRadians = std::acos(cosTheta);
    double angleDegrees = angleRadians * (180.0 / M_PI);

    // Determine the direction of rotation using the cross product
    double crossProductZ =
        normLine1.x() * normLine2.y() - normLine1.y() * normLine2.x();

    if (button->flipTextRotation) {
      if (crossProductZ < 0) {
        angleDegrees = -angleDegrees;
      }
    } else {
      if (crossProductZ > 0) {
        angleDegrees = -angleDegrees;
      }
    }
    */

    // Get button's circle properties
    double x = button->center_projected_point.x;
    double y = button->center_projected_point.y;
    double radius = button->radius;
    cv::Scalar btn_color = button->btn_color;  // BGR color

    // Convert OpenCV BGR color to Cairo RGB
    double red = btn_color[0] / 255.0;
    double green = btn_color[1] / 255.0;
    double blue = btn_color[2] / 255.0;

    // Set Cairo color (RGB)
    cr->set_source_rgb(red, green, blue);

    // Draw and fill the circle (button) in Cairo
    cr->arc(x, y, button->radius, 0,
            2 * M_PI);  // Circle with center (x, y) and radius
    cr->fill();         // Fill the circle with color

    // Draw the text at the center of the circle
    cr->set_source_rgb(1.0, 1.0,
                       1.0);  // Set text color to white (or any other color)
    cr->select_font_face("Arial", Cairo::FONT_SLANT_NORMAL,
                         Cairo::FONT_WEIGHT_BOLD);
    cr->set_font_size(24);  // Set font size

    // Calculate the width and height of the text to center it
    std::string text = button->text;
    Cairo::TextExtents extents;  // Declare the TextExtents object
    cr->get_text_extents(text,
                         extents);  // Pass the object to get the text extents

    // Use the extents to position the text at the center of the circle
    double text_width = extents.width;
    double text_height = extents.height;

    // Move the text to the center of the circle (adjusting for text size)
    cr->move_to(x - text_width / 2, y + text_height / 2);
    // Draw the text
    cr->show_text(text);
  }
  // Step 4: Convert Cairo image surface data back to OpenCV Mat
  unsigned char *data = surface->get_data();

  // Convert the Cairo surface data (RGB) to OpenCV Mat (BGR)
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int offset = (y * width + x) * 4;
      layer->at<cv::Vec3b>(y, x)[0] = data[offset + 2];  // B
      layer->at<cv::Vec3b>(y, x)[1] = data[offset + 1];  // G
      layer->at<cv::Vec3b>(y, x)[2] = data[offset + 0];  // R
    }
  }
}

void Projector::updateButtons(
    const std::vector<tuni_whitegoods_msgs::Button> &buttons,
    std::shared_ptr<cv::Mat> layer) {
  int width = layer->cols;
  int height = layer->rows;

  auto surface =
      Cairo::ImageSurface::create(Cairo::Format::FORMAT_RGB24, width, height);
  auto cr = Cairo::Context::create(surface);

  for (auto &button : buttons) {
    // Get button's circle properties
    double x = button.center.x;
    double y = button.center.y;
    double radius = button.radius;

    double red = button.color.r * 255;
    double green = button.color.g * 255;
    double blue = button.color.b * 255;

    // Set Cairo color (RGB)
    cr->set_source_rgb(red, green, blue);

    // Draw and fill the circle (button) in Cairo
    cr->arc(x, y, button.radius, 0,
            2 * M_PI);  // Circle with center (x, y) and radius
    cr->fill();         // Fill the circle with color

    // Draw the text at the center of the circle
    cr->set_source_rgb(1.0, 1.0,
                       1.0);  // Set text color to white (or any other color)
    cr->select_font_face("Arial", Cairo::FONT_SLANT_NORMAL,
                         Cairo::FONT_WEIGHT_BOLD);
    cr->set_font_size(24);  // Set font size

    // Calculate the width and height of the text to center it
    std::string text = button.text;
    Cairo::TextExtents extents;  // Declare the TextExtents object
    cr->get_text_extents(text,
                         extents);  // Pass the object to get the text extents

    // Use the extents to position the text at the center of the circle
    double text_width = extents.width;
    double text_height = extents.height;

    // Move the text to the center of the circle (adjusting for text size)
    cr->move_to(x - text_width / 2, y + text_height / 2);
    // Draw the text
    cr->show_text(text);
  }
  // Step 4: Convert Cairo image surface data back to OpenCV Mat
  unsigned char *data = surface->get_data();

  // Convert the Cairo surface data (RGB) to OpenCV Mat (BGR)
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int offset = (y * width + x) * 4;
      layer->at<cv::Vec3b>(y, x)[0] = data[offset + 2];  // B
      layer->at<cv::Vec3b>(y, x)[1] = data[offset + 1];  // G
      layer->at<cv::Vec3b>(y, x)[2] = data[offset + 0];  // R
    }
  }
}

void Projector::updateBorders(
    const std::vector<std::shared_ptr<StaticBorder>> &borders,
    std::shared_ptr<cv::Mat> layer) {
  for (auto &border : borders) {
    cv::rectangle(
        *layer, border->top_left_proj_point, border->bottom_right_proj_point,
        cv::Scalar(border->border_color.b * 255, border->border_color.g * 255,
                   border->border_color.r * 255),
        border->thickness * 2, cv::LINE_8);
  }
}

void Projector::updateBorders(
    const std::vector<tuni_whitegoods_msgs::StaticBorder> &borders,
    std::shared_ptr<cv::Mat> layer) {
  for (auto &border : borders) {
    cv::Point top_left(border.top_left.x, border.top_left.y);
    cv::Point bottom_right(border.bottom_right.x, border.bottom_right.y);

    cv::rectangle(*layer, top_left, bottom_right,
                  cv::Scalar(border.color.b * 255, border.color.g * 255,
                             border.color.r * 255),
                  border.thickness * 2, cv::LINE_8);
  }
}

void Projector::updateHands(const std::vector<std::shared_ptr<Hand>> &hands) {}

void Projector::updateDisplayAreas(
    const std::vector<std::shared_ptr<DisplayArea>> &zones) {
  for (auto &zone : zones) {
    if (!(zone->name == "projector" || zone->name == "camera")) {
      if (id_ == zone->projector_id_) {
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
        bool visibility = layers[zone->name].visible;

        layers[zone->name] = {
            std::make_shared<cv::Mat>(cv::Mat::zeros(
                projector_resolution[1], projector_resolution[0], CV_8UC3)),
            visibility};

        std::vector<std::shared_ptr<Button>> buttons;
        zone->fetchButtons(buttons);
        if (!buttons.empty()) {
          updateButtons(buttons, layers[zone->name].mat);
        }

        ros::Duration(0.1).sleep();

        std::vector<std::shared_ptr<StaticBorder>> borders;
        zone->fetchBorders(borders);
        if (!borders.empty()) {
          updateBorders(borders, layers[zone->name].mat);
        }

        cv::polylines(*layers[zone->name].mat, rectanglePoints, true,
                      zone->color, 10, cv::LINE_8);

        if (!zone->instructions.empty()) {
          ROS_INFO("Drawing instructions");
          cv::putText(*layers[zone->name].mat, zone->instructions,
                      cv::Point((tl.x + bl.x) / 3.9, (tl.y + tr.y) / 2.2),
                      TEXT_FACE, TEXT_SCALE_TITLE, cv::Scalar(255, 255, 255),
                      TEXT_THICKNESS, cv::LINE_AA);
        }
      }
    }
  }
  combined = layers["background"].mat->clone();
  for (auto it = layers.begin(); it != layers.end(); ++it) {
    if (it->second.visible) {
      cv::bitwise_or(combined, *(it->second.mat), combined);
    }
  }

  project_image();
}

void Projector::update(
    const std::vector<tuni_whitegoods_msgs::ZoneState> &msg) {
  for (const auto &zone : msg) {
    if (std::to_string(id_) == zone.projector_id) {
      // bool visibility = layers[zone.name].visible;

      if (layers.find(zone.name) == layers.end()) {
        layers[zone.name] = {
            std::make_shared<cv::Mat>(cv::Mat::zeros(
                projector_resolution[1], projector_resolution[0], CV_8UC3)),
            true};
      } else {
        layers[zone.name].mat->setTo(cv::Scalar(0, 0, 0));
      }

      // Buttons
      if (!zone.buttons.empty()) {
        updateButtons(zone.buttons, layers[zone.name].mat);
      }

      // Borders
      if (!zone.borders.empty()) {
        updateBorders(zone.borders, layers[zone.name].mat);
      }

      // Area itself
      cv::Point tl(zone.frame_area[0].x, zone.frame_area[0].y);
      cv::Point tr(zone.frame_area[1].x, zone.frame_area[1].y);
      cv::Point br(zone.frame_area[2].x, zone.frame_area[2].y);
      cv::Point bl(zone.frame_area[3].x, zone.frame_area[3].y);
      std::vector<cv::Point> rectanglePoints = {tl, tr, br, bl};
      cv::Scalar color(zone.color.b * 255, zone.color.g * 255,
                       zone.color.r * 255);
      cv::polylines(*layers[zone.name].mat, rectanglePoints, true, color, 10,
                    cv::LINE_8);

      // Instructions
      if (!zone.instructions.empty()) {
        cv::putText(*layers[zone.name].mat, zone.instructions,
                    cv::Point((tl.x + bl.x) / 3.9, (tl.y + tr.y) / 2.2),
                    TEXT_FACE, TEXT_SCALE_TITLE, cv::Scalar(255, 255, 255),
                    TEXT_THICKNESS, cv::LINE_AA);
      }
    }
  }

  combined.setTo(cv::Scalar(0, 0, 0));
  for (auto it = layers.begin(); it != layers.end(); ++it) {
    if (it->second.visible) {
      cv::bitwise_or(combined, *(it->second.mat), combined);
    }
  }

  project_image();
}

void Projector::project_image() {
  cv::imshow(window_name, combined);
  cv::waitKey(50);
}

void Projector::tableDetectionCallback(
    const tuni_whitegoods_msgs::DynamicArea::ConstPtr &msg) {
  top_left_moving_table_x = msg->top_left[0];
  top_left_moving_table_y = msg->top_left[1];

  top_right_moving_table_x = msg->top_right[0];
  top_right_moving_table_y = msg->top_right[1];

  bottom_right_moving_table_x = msg->bottom_right[0];
  bottom_right_moving_table_y = msg->bottom_right[1];

  bottom_left_moving_table_x = msg->bottom_left[0];
  bottom_left_moving_table_y = msg->bottom_left[1];

  height_moving_table = top_left_moving_table_y - bottom_left_moving_table_y;
  width_moving_table = top_right_moving_table_x - top_left_moving_table_x;
}
