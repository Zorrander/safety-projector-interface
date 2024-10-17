#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

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

    cv::Mat homography_matrix =
        cv::findHomography(original_position, new_position);

    // res.transform = ;

    return true;
  }

  ros::NodeHandle* nh_;
  ros::ServiceServer table_point_transform_service_;
  std::vector<cv::Point2f> original_position;
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