#include <ros/ros.h>

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
  }

 private:
  /**
   * @brief      Service for transforming 3D points to 2D.
   *
   * This service takes a 3D point in the camera coordinates frame and projects
   * it onto its equivalent in a 2D image.
   *
   * @param      req   Request object containing (x, y, z) world coordinates.
   * @param      res   Response object containing (u, v) pixel coordinates
   *
   * @return     true if the service call was successful, false otherwise.
   */
  bool transformTablePointCallback(
      tuni_whitegoods_msgs::TransformMovingTable::Request& req,
      tuni_whitegoods_msgs::TransformMovingTable::Response& res) {
    // Define the points (2D points in both sets)
    std::vector<cv::Point2f> original_position = {
        cv::Point2f(0, 0), cv::Point2f(1, 1), cv::Point2f(2, 1),
        cv::Point2f(2, 2)};

    std::vector<cv::Point2f> new_position = {
        cv::Point2f(req.table.top_left[0], req.table.top_left[1]),
        cv::Point2f(req.table.top_right[0], req.table.top_right[1]),
        cv::Point2f(req.table.bottom_right[0], req.table.bottom_right[1]),
        cv::Point2f(req.table.bottom_left[0], req.table.bottom_left[1])};

    // Matrix to store the affine transformation
    cv::Mat affineMatrix(2, 3, CV_64F);

    // Inliers mask (if you want to know which points were considered inliers)
    std::vector<uchar> inliers;

    // Estimate the affine transformation
    affineMatrix =
        cv::estimateAffine2D(original_position, new_position, inliers);

    // Extract the rotation and translation components
    cv::Mat rotation =
        affineMatrix(cv::Rect(0, 0, 2, 2));  // Extract 2x2 rotation matrix
    cv::Mat translation =
        affineMatrix(cv::Rect(2, 0, 1, 2));  // Extract 2x1 translation vector

    // Output the matrix
    std::cout << "Affine Matrix: \n" << affineMatrix << std::endl;
    std::cout << "Rotation: \n" << rotation << std::endl;
    std::cout << "Translation: \n" << translation << std::endl;

    return true;
  }

  ros::NodeHandle* nh_;
  ros::ServiceServer table_point_transform_service_;
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