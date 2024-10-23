#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "tuni_whitegoods_msgs/TransformMovingTable.h"

/**
 * @brief      Projector pixels transformations.
 *
 * This class holds two service definitions. One to project pixel from the
 * camera to the projector and the other to project projected pixel to
 * their equivalent on the camera screen.
 */
class TransformTablePointServer {
 public:
  /**
   * @brief      Instantiates two service servers.
   *
   * Instantiates the transformation servers and loads the necessary homography
   * matrices.
   */
  TransformTablePointServer(ros::NodeHandle* nh) : nh_(nh) {
    table_point_transform_service_ = nh_->advertiseService(
        "transform_table_server",
        &TransformTablePointServer::transformTablePointCallback, this);

    std::string display_areas_calibration_file;
    if (!nh_->getParam("display_areas_calibration_file",
                       display_areas_calibration_file)) {
      ROS_ERROR(
          "display_areas calibration file is missing from configuration.");
    }

    YAML::Node display_areas_calibration =
        YAML::LoadFile(display_areas_calibration_file);

    ros::Duration(2.0).sleep();

    for (YAML::const_iterator it = display_areas_calibration.begin();
         it != display_areas_calibration.end(); ++it) {
      std::string area_name = it->first.as<std::string>();
      YAML::Node area_node = it->second;

      if (area_name == "moving_table") {
        original_corners.push_back(
            cv::Point2f(area_node["top_left"]["x"].as<double>(),
                        area_node["top_left"]["y"].as<double>()));

        original_corners.push_back(
            cv::Point2f(area_node["top_right"]["x"].as<double>(),
                        area_node["top_right"]["y"].as<double>()));

        original_corners.push_back(
            cv::Point2f(area_node["bottom_right"]["x"].as<double>(),
                        area_node["bottom_right"]["y"].as<double>()));

        original_corners.push_back(
            cv::Point2f(area_node["bottom_left"]["x"].as<double>(),
                        area_node["bottom_left"]["y"].as<double>()));
      }
    }
  }

 private:
  bool transformTablePointCallback(
      tuni_whitegoods_msgs::TransformMovingTable::Request& req,
      tuni_whitegoods_msgs::TransformMovingTable::Response& res) {
    if (original_position.empty()) {
      original_position = {
          cv::Point2f(req.table.top_left[0], req.table.top_left[1]),
          cv::Point2f(req.table.top_right[0], req.table.top_right[1]),
          cv::Point2f(req.table.bottom_right[0], req.table.bottom_right[1]),
          cv::Point2f(req.table.bottom_left[0], req.table.bottom_left[1])};
    }

    std::vector<cv::Point2f> new_position = {
        cv::Point2f(req.table.top_left[0], req.table.top_left[1]),
        cv::Point2f(req.table.top_right[0], req.table.top_right[1]),
        cv::Point2f(req.table.bottom_right[0], req.table.bottom_right[1]),
        cv::Point2f(req.table.bottom_left[0], req.table.bottom_left[1])};

    cv::Mat moving_table_homography =
        cv::findHomography(original_position, new_position);

    std::vector<cv::Point2f> transformed_corners;
    cv::perspectiveTransform(original_corners, transformed_corners,
                             moving_table_homography);

    res.table_corners.top_left[0] =
        static_cast<double>(transformed_corners[0].x);
    res.table_corners.top_left[1] =
        static_cast<double>(transformed_corners[0].y);

    res.table_corners.top_right[0] =
        static_cast<double>(transformed_corners[1].x);
    res.table_corners.top_right[1] =
        static_cast<double>(transformed_corners[1].y);

    res.table_corners.bottom_right[0] =
        static_cast<double>(transformed_corners[2].x);
    res.table_corners.bottom_right[1] =
        static_cast<double>(transformed_corners[2].y);

    res.table_corners.bottom_left[0] =
        static_cast<double>(transformed_corners[3].x);
    res.table_corners.bottom_left[1] =
        static_cast<double>(transformed_corners[3].y);

    return true;
  }

  ros::NodeHandle* nh_;
  ros::ServiceServer table_point_transform_service_;
  std::vector<cv::Point2f> original_position;
  std::vector<cv::Point2f> original_corners;
};

/**
 * @brief      { function_description }
 *
 * @param[in]  argc  The count of arguments
 * @param      argv  The arguments array
 *
 * @return     { description_of_the_return_value }
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "transform_table_server");
  ros::NodeHandle nh;
  TransformTablePointServer server(&nh);

  ros::spin();

  return 0;
}