#include "tuni_whitegoods_view/project_view.h"

#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>
#include <boost/algorithm/clamp.hpp>
#include <cmath>

#include "tf2/utils.h"

using namespace std;

Projector::Projector(ros::NodeHandle *nh, int id)
    : nh_(nh), id_(id) {
  if (!ros::param::get("shiftX", shift)) {
    shift = 0;  // Default value
    ROS_WARN("Parameter 'shiftX' not found, using default value 0.");
  } else {
    shift = shift * id_;
    ROS_INFO("Using shift %d", shift);
  }
  ros::param::get("projector_resolution", projector_resolution);

  layers["background"] = {
      std::make_shared<cv::Mat>(cv::Mat::zeros(
          projector_resolution[1], projector_resolution[0], CV_8UC3)),
      true};

  table_detection_sub =
      nh_->subscribe("/odin/projector_interface/moving_table/transform", 5,
                     &Projector::tableDetectionCallback, this);


  combined = layers["background"].mat->clone();

  ROS_INFO("ProjectorView running");
}

Projector::~Projector() {
  cv::destroyWindow(window_name);
}

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
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  cv::moveWindow(window_name, shift, 0);
  cv::setWindowProperty(window_name, cv::WND_PROP_FULLSCREEN,
                        cv::WINDOW_FULLSCREEN);
  project_image();
  ROS_INFO("init done");
}

void Projector::moveWindow(){
  cv::moveWindow(window_name, shift, 0);
  project_image();
}

void Projector::updateButtons(
    const std::vector<std::shared_ptr<Button>> &buttons,
    std::shared_ptr<cv::Mat> layer) {
  for (auto &button : buttons) {
    cv::circle(*layer, button->center_projected_point, button->radius,
               button->btn_color, -1);
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;

    cv::Size textSize =
        cv::getTextSize(button->text, fontFace, button->fontScale,
                        button->thickness, &button->baseline);

    cv::Mat textImage = cv::Mat::zeros(textSize.height + button->baseline,
                                       textSize.width, CV_8UC3);
    cv::putText(textImage, button->text,
                cv::Point(button->origin_text_x, button->origin_text_y),
                fontFace, button->fontScale, button->txt_color,
                button->thickness);

    // rectangle around that to see what the projection area would be if
    // straight
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
      if (crossProductZ > 0) {
        angleDegrees = -angleDegrees;
      }
    } else {
      if (crossProductZ < 0) {
        angleDegrees = -angleDegrees;
      }
    }
    // Compute rotation matrix
    // Estimate the bounding box for the rotated text
    int padding = static_cast<int>(std::sqrt(
        textSize.width * textSize.width + textSize.height * textSize.height));

    cv::Mat paddedTextImage = cv::Mat::zeros(
        textImage.rows + padding, textImage.cols + padding, textImage.type());

    // Center the original text in the new padded image
    textImage.copyTo(paddedTextImage(
        cv::Rect(padding / 2, padding / 2, textImage.cols, textImage.rows)));

    // Update the rotation matrix to rotate around the center of the new
    // padded image
    cv::Point2f center(paddedTextImage.cols / 2.0f,
                       paddedTextImage.rows / 2.0f);
    cv::Mat rotationMatrix = cv::getRotationMatrix2D(center, angleDegrees, 1.0);

    cv::Mat rotatedText;
    cv::warpAffine(paddedTextImage, rotatedText, rotationMatrix,
                   paddedTextImage.size(), cv::INTER_LANCZOS4,
                   cv::BORDER_TRANSPARENT);

    // Calculate the new ROI based on the expanded rotated text dimensions
    cv::Rect roi(button->center_projected_point.x - rotatedText.cols / 2,
                 button->center_projected_point.y - rotatedText.rows / 2,
                 rotatedText.cols, rotatedText.rows);

    // Ensure ROI stays within the bounds of `layer`
    int x = std::max(0, roi.x);
    int y = std::max(0, roi.y);
    int width = std::min(roi.width, layer->cols - x);
    int height = std::min(roi.height, layer->rows - y);

    cv::Rect validRoi(x, y, width, height);

    // Copy the rotated text into the layer, using the valid ROI to handle
    // boundaries
    rotatedText(cv::Rect(0, 0, validRoi.width, validRoi.height))
        .copyTo((*layer)(validRoi), rotatedText);
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

void Projector::updateHands(const std::vector<std::shared_ptr<Hand>> &hands) {}

void Projector::updateDisplayAreas(
    const std::vector<std::shared_ptr<DisplayArea>> &zones) {
  ROS_INFO("updateDisplayAreas");
  for (auto &zone : zones) {
    if (!(zone->name == "projector" || zone->name == "camera")) {
      if(id_ == zone->projector_id_) {
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

        cv::polylines(*layers[zone->name].mat, rectanglePoints, true,
                      cv::Scalar(255), 10, cv::LINE_8);

        std::vector<std::shared_ptr<Button>> buttons;
        zone->fetchButtons(buttons);
        if (!buttons.empty()) { 
          updateButtons(buttons, layers[zone->name].mat);
        }

        std::vector<std::shared_ptr<StaticBorder>> borders;
        zone->fetchBorders(borders);
        if (!borders.empty()) { 
          updateBorders(borders, layers[zone->name].mat);
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

void Projector::project_image() {
  cv::imshow(window_name, combined);
  cv::waitKey(1);
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
