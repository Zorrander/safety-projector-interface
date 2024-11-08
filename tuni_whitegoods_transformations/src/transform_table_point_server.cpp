#include <ros/ros.h>

#include <Eigen/Dense>
#include <boost/algorithm/clamp.hpp>
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

    ros::param::get("/moving_table_corner_homography",
                    moving_table_corner_homography_array);
    moving_table_corner_homography =
        cv::Matx33d(moving_table_corner_homography_array.data());
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
      cv::perspectiveTransform(original_position, original_corners,
                               moving_table_corner_homography);
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

    // Calculate the center
    double centerX = (transformed_corners[0].x + transformed_corners[1].x +
                      transformed_corners[2].x + transformed_corners[3].x) /
                     4.0;
    double centerY = (transformed_corners[0].y + transformed_corners[1].y +
                      transformed_corners[2].y + transformed_corners[3].y) /
                     4.0;

    cv::Point2f center(centerX, centerY);

    // Calculate side lengths
    double width1 = std::hypot(
        transformed_corners[1].x - transformed_corners[0].x,
        transformed_corners[1].y - transformed_corners[0].y);  // Top side
    double width2 = std::hypot(
        transformed_corners[2].x - transformed_corners[3].x,
        transformed_corners[2].y - transformed_corners[3].y);  // Bottom side
    double height1 = std::hypot(
        transformed_corners[3].x - transformed_corners[0].x,
        transformed_corners[3].y - transformed_corners[0].y);  // Left side
    double height2 = std::hypot(
        transformed_corners[2].x - transformed_corners[1].x,
        transformed_corners[2].y - transformed_corners[1].y);  // Right side

    // Find maximum width and height
    double max_width = std::max(width1, width2);
    double max_height = std::max(height1, height2);

    // Determine if it's portrait or landscape
    bool isPortrait = (max_height > max_width);

    if (isPortrait) {
      // Swap if needed so that max_height is the vertical and max_width is
      // horizontal
      std::swap(max_width, max_height);
    }
    cv::Size2f size(max_width, max_height);

    int top_left_straight_table_x, top_left_straight_table_y,
        top_right_straight_table_x, top_right_straight_table_y;

    top_left_straight_table_x = centerX - max_width / 2;
    top_left_straight_table_y = centerY - max_height / 2;
    top_right_straight_table_x = centerX + max_width / 2;
    top_right_straight_table_y = centerY - max_height / 2;

    Eigen::Vector2d point1(top_left_straight_table_x,
                           top_left_straight_table_y);
    Eigen::Vector2d point2(top_right_straight_table_x,
                           top_right_straight_table_y);
    Eigen::Vector2d point3(transformed_corners[0].x, transformed_corners[0].y);
    Eigen::Vector2d point4(transformed_corners[1].x, transformed_corners[1].y);

    // Calculate rotation angle

    Eigen::Vector2d direction1 = point2 - point1;
    Eigen::Vector2d direction2 = point4 - point3;

    Eigen::Vector2d normLine1 = direction1.normalized();
    Eigen::Vector2d normLine2 = direction2.normalized();

    double cosTheta = normLine1.dot(normLine2);
    cosTheta = boost::algorithm::clamp(cosTheta, -1.0, 1.0);

    double angleRadians = std::acos(cosTheta);
    double angleDegrees = angleRadians * (180.0 / M_PI);

    double crossProductZ =
        normLine1.x() * normLine2.y() - normLine1.y() * normLine2.x();
    if (crossProductZ < 0) {
      angleDegrees = -angleDegrees;
    }

    cv::RotatedRect movingTable(center, size, angleDegrees);

    cv::Point2f vertices[4];
    movingTable.points(vertices);

    res.table_corners.top_left[0] = static_cast<double>(vertices[0].x);
    res.table_corners.top_left[1] = static_cast<double>(vertices[0].y);

    res.table_corners.top_right[0] = static_cast<double>(vertices[1].x);
    res.table_corners.top_right[1] = static_cast<double>(vertices[1].y);

    res.table_corners.bottom_right[0] = static_cast<double>(vertices[2].x);
    res.table_corners.bottom_right[1] = static_cast<double>(vertices[2].y);

    res.table_corners.bottom_left[0] = static_cast<double>(vertices[3].x);
    res.table_corners.bottom_left[1] = static_cast<double>(vertices[3].y);

    return true;
  }

  ros::NodeHandle* nh_;
  ros::ServiceServer table_point_transform_service_;
  std::vector<cv::Point2f> original_position;
  std::vector<cv::Point2f> original_corners;

  cv::Matx33d moving_table_corner_homography;
  std::vector<double> moving_table_corner_homography_array;
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