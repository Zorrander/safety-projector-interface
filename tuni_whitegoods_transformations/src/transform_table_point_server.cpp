#include <ros/ros.h>
#include <std_msgs/Int32.h>

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
    sub_max_width =
        nh_->subscribe("/odin/object_detection/set_max_width_table", 1,
                       &TransformTablePointServer::widthCallback, this);

    sub_max_height =
        nh_->subscribe("/odin/object_detection/set_max_height_table", 1,
                       &TransformTablePointServer::heightCallback, this);

    sub_angle = nh_->subscribe("/odin/object_detection/set_angle_table", 1,
                               &TransformTablePointServer::angleCallback, this);

    sub_center =
        nh_->subscribe("/odin/object_detection/set_center_table", 1,
                       &TransformTablePointServer::centerCallback, this);

    original_position = {cv::Point2f(775, 81), cv::Point2f(809, 90),
                         cv::Point2f(795, 119), cv::Point2f(757, 109)};

    original_corners = {cv::Point2f(795, 119), cv::Point2f(856, 141),
                        cv::Point2f(803, 247), cv::Point2f(728, 221)};

    original_table_centerX = (original_corners[0].x + original_corners[1].x +
                              original_corners[2].x + original_corners[3].x) /
                             4.0;
    original_table_centerY = (original_corners[0].y + original_corners[1].y +
                              original_corners[2].y + original_corners[3].y) /
                             4.0;
    original_marker_centerX =
        (original_position[0].x + original_position[1].x +
         original_position[2].x + original_position[3].x) /
        4.0;
    original_marker_centerY =
        (original_position[0].y + original_position[1].y +
         original_position[2].y + original_position[3].y) /
        4.0;

    center_distance =
        std::hypot(original_table_centerX - original_marker_centerX,
                   original_table_centerY - original_marker_centerY);

    // Calculate side lengths
    double original_width1 =
        std::hypot(original_corners[1].x - original_corners[0].x,
                   original_corners[1].y - original_corners[0].y);
    double original_width2 =
        std::hypot(original_corners[2].x - original_corners[3].x,
                   original_corners[2].y - original_corners[3].y);
    double original_height1 =
        std::hypot(original_corners[3].x - original_corners[0].x,
                   original_corners[3].y - original_corners[0].y);
    double original_height2 =
        std::hypot(original_corners[2].x - original_corners[1].x,
                   original_corners[2].y - original_corners[1].y);

    // Find maximum width and height
    original_max_width = std::max(original_width1, original_width2);
    original_max_height = std::max(original_height1, original_height2);

    // Determine if it's portrait or landscape
    bool isPortrait = (original_max_height > original_max_width);

    if (isPortrait) {
      // Swap if needed so that max_height is the vertical and max_width is
      // horizontal
      std::swap(original_max_width, original_max_height);
    }

    // original_max_width = 100;
    // original_max_height = 100;
  }

  cv::Mat moving_table_homography;

 private:
  bool transformTablePointCallback(
      tuni_whitegoods_msgs::TransformMovingTable::Request& req,
      tuni_whitegoods_msgs::TransformMovingTable::Response& res) {
    std::vector<cv::Point2f> new_position = {
        cv::Point2f(req.table.top_left[0], req.table.top_left[1]),
        cv::Point2f(req.table.top_right[0], req.table.top_right[1]),
        cv::Point2f(req.table.bottom_right[0], req.table.bottom_right[1]),
        cv::Point2f(req.table.bottom_left[0], req.table.bottom_left[1])};

    std::vector<cv::Point2f> rect1_edges;
    for (size_t i = 0; i < 4; ++i) {
      cv::Point2f p1 = new_position[i];
      cv::Point2f p2 =
          new_position[(i + 1) % 4];   // Wrap around to the first point
      rect1_edges.push_back(p2 - p1);  // Store edge vectors
    }
    // Normalize the edge directions for comparison
    std::vector<cv::Point2f> normalized_edges;
    for (const auto& edge : rect1_edges) {
      float length = std::hypot(edge.x, edge.y);
      normalized_edges.emplace_back(edge.x / length, edge.y / length);
    }

    moving_table_homography =
        cv::getPerspectiveTransform(original_position, new_position);

    std::vector<cv::Point2f> transformed_corners;
    cv::perspectiveTransform(original_corners, transformed_corners,
                             moving_table_homography);

    double marker_centerX = (new_position[0].x + new_position[1].x +
                             new_position[2].x + new_position[3].x) /
                            4.0;
    double marker_centerY = (new_position[0].y + new_position[1].y +
                             new_position[2].y + new_position[3].y) /
                            4.0;

    // Calculate the center
    double centerX = (transformed_corners[0].x + transformed_corners[1].x +
                      transformed_corners[2].x + transformed_corners[3].x) /
                     4.0;
    double centerY = (transformed_corners[0].y + transformed_corners[1].y +
                      transformed_corners[2].y + transformed_corners[3].y) /
                     4.0;

    /*test*/

    // Centers of the detected and inferred rectangles
    double x1 = marker_centerX;
    double y1 = marker_centerY;
    double x2_prime = centerX;
    double y2_prime = centerY;

    // Calculate vector and current distance
    double dx = x2_prime - x1;
    double dy = y2_prime - y1;
    double current_distance = std::hypot(dx, dy);

    // Normalize the vector
    double unit_dx = dx / current_distance;
    double unit_dy = dy / current_distance;

    // Set the fixed distance
    double fixed_distance = center_distance + gui_center;

    // Scale the vector to the fixed distance
    double scaled_dx = unit_dx * fixed_distance;
    double scaled_dy = unit_dy * fixed_distance;

    // Calculate the new center coordinates
    double x2 = x1 + scaled_dx;
    double y2 = y1 + scaled_dy;
    cv::Point2f center(x2, y2);

    // cv::Point2f center(centerX, centerY);
    /*
    double x1 = marker_centerX;
    double y1 = marker_centerY;
    double dx = new_position[0].x - new_position[3].x;
    double dy = new_position[0].y - new_position[3].y;
    double scaled_dx = dx * 3;
    double scaled_dy = dy * 2;
    double x2 = new_position[3].x + scaled_dx;
    double y2 = new_position[3].y - scaled_dy;
    cv::Point2f center(x2, y2);
    */
    // Calculate side lengths
    double width1 =
        std::hypot(transformed_corners[1].x - transformed_corners[0].x,
                   transformed_corners[1].y - transformed_corners[0].y);
    double width2 =
        std::hypot(transformed_corners[2].x - transformed_corners[3].x,
                   transformed_corners[2].y - transformed_corners[3].y);
    double height1 =
        std::hypot(transformed_corners[3].x - transformed_corners[0].x,
                   transformed_corners[3].y - transformed_corners[0].y);
    double height2 =
        std::hypot(transformed_corners[2].x - transformed_corners[1].x,
                   transformed_corners[2].y - transformed_corners[1].y);

    // Find maximum width and height
    double max_width = original_max_width + gui_max_width;
    double max_height = original_max_height + gui_max_height;

    // Determine if it's portrait or landscape
    bool isPortrait = (std::max(height1, height2) > std::max(width1, width2));

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

    // Calculate the edge directions of rotatedRect2
    std::vector<cv::Point2f> rect2_edges;
    for (size_t i = 0; i < 4; ++i) {
      cv::Point2f p1 = vertices[i];
      cv::Point2f p2 = vertices[(i + 1) % 4];
      rect2_edges.push_back(p2 - p1);
    }

    // Adjust the orientation of rotatedRect2 by modifying its angle
    double new_angle = movingTable.angle;
    for (const auto& edge : rect2_edges) {
      float length = std::hypot(edge.x, edge.y);
      cv::Point2f normalized_edge(edge.x / length, edge.y / length);

      // Check for parallelism with the edges of rect1
      for (const auto& rect1_edge : normalized_edges) {
        // Check if the vectors are aligned (dot product close to 1 or -1)
        float dot_product =
            normalized_edge.x * rect1_edge.x + normalized_edge.y * rect1_edge.y;

        if (std::fabs(dot_product) > 0.90) {
          // Adjust the angle of rotatedRect2 to align it
          new_angle = atan2(rect1_edge.y, rect1_edge.x) * 180.0 / CV_PI;
          break;
        }
      }
    }
    cv::RotatedRect alignedMovingTable(center, size, new_angle + gui_angle);

    cv::Point2f alignedVertices[4];
    alignedMovingTable.points(alignedVertices);

    res.table_corners.top_left[0] = static_cast<double>(alignedVertices[0].x);
    res.table_corners.top_left[1] = static_cast<double>(alignedVertices[0].y);

    res.table_corners.top_right[0] = static_cast<double>(alignedVertices[1].x);
    res.table_corners.top_right[1] = static_cast<double>(alignedVertices[1].y);

    res.table_corners.bottom_right[0] =
        static_cast<double>(alignedVertices[2].x);
    res.table_corners.bottom_right[1] =
        static_cast<double>(alignedVertices[2].y);

    res.table_corners.bottom_left[0] =
        static_cast<double>(alignedVertices[3].x);
    res.table_corners.bottom_left[1] =
        static_cast<double>(alignedVertices[3].y);

    res.table_corners.rotation_angle = new_angle + gui_angle;

    return true;
  }

  void widthCallback(const std_msgs::Int32::ConstPtr& msg) {
    gui_max_width = msg->data;
  }
  void heightCallback(const std_msgs::Int32::ConstPtr& msg) {
    gui_max_height = msg->data;
  }
  void angleCallback(const std_msgs::Int32::ConstPtr& msg) {
    gui_angle = msg->data;
  }
  void centerCallback(const std_msgs::Int32::ConstPtr& msg) {
    gui_center = msg->data;
  }

  ros::NodeHandle* nh_;
  ros::ServiceServer table_point_transform_service_;
  std::vector<cv::Point2f> original_position;
  std::vector<cv::Point2f> original_corners;
  double original_table_centerX;
  double original_table_centerY;
  double original_marker_centerX;
  double original_marker_centerY;
  double center_distance;
  double original_max_width;
  double original_max_height;
  cv::Size2f original_size;
  double gui_max_width;
  double gui_max_height;
  double gui_angle;
  double gui_center;
  ros::Subscriber sub_max_width;
  ros::Subscriber sub_max_height;
  ros::Subscriber sub_angle;
  ros::Subscriber sub_center;
  ros::Subscriber sub_threshold_detection_table;
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