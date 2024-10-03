#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include "tuni_whitegoods_msgs/TransformPixelToProjection.h"

/**
 * @brief      This class describes a transform projector point server.
 */
class TransformProjectorPointServer {
 public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param      nh    { parameter_description }
   */
  TransformProjectorPointServer(ros::NodeHandle* nh) : nh_(nh) {
    projector_point_transform_service_ = nh_->advertiseService(
        "transform_point_to_project",
        &TransformProjectorPointServer::transformProjectorPointCallback, this);
    reverse_projector_point_transform_service_ = nh_->advertiseService(
        "reverse_transform_point_to_project",
        &TransformProjectorPointServer::reverseTransformProjectorPointCallback,
        this);
    // ros::param::get("border_homography", border_homography_array);
    ros::param::get("button_homography", button_homography_array);
    // border_homography =  cv::Matx33d(border_homography_array.data());
    border_homography = loadHomography("/home/odin3/Desktop/hmg.yaml");
    button_homography = cv::Matx33d(button_homography_array.data());
  }

 private:
  /**
   * @brief      Loads a homography.
   *
   * @param[in]  filename  The filename
   *
   * @return     { description_of_the_return_value }
   */
  cv::Mat loadHomography(const std::string& filename) {
    cv::Mat homography;

    // Open the file storage
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (fs.isOpened()) {
      // Read the homography matrix from the file
      fs["homography"] >> homography;
      fs.release();  // Release the file
      std::cout << "Homography loaded from " << filename << std::endl;
    } else {
      std::cerr << "Error: Unable to open file for reading: " << filename
                << std::endl;
    }

    return homography;
  }

  /**
   * @brief      { function_description }
   *
   * @param      req   The request
   * @param      res   The resource
   *
   * @return     { description_of_the_return_value }
   */
  bool transformProjectorPointCallback(
      tuni_whitegoods_msgs::TransformPixelToProjection::Request& req,
      tuni_whitegoods_msgs::TransformPixelToProjection::Response& res) {
    std::vector<cv::Point2f> cameraPoint, projectorPoint;

    cv::Point2f input_point(req.u, req.v);
    cameraPoint.push_back(input_point);
    cv::perspectiveTransform(cameraPoint, projectorPoint, border_homography);
    res.u_prime = projectorPoint[0].x;
    res.v_prime = projectorPoint[0].y;

    return true;
  }

  /**
   * @brief      { function_description }
   *
   * @param      req   The request
   * @param      res   The resource
   *
   * @return     { description_of_the_return_value }
   */
  bool reverseTransformProjectorPointCallback(
      tuni_whitegoods_msgs::TransformPixelToProjection::Request& req,
      tuni_whitegoods_msgs::TransformPixelToProjection::Response& res) {
    std::vector<cv::Point2f> cameraPoint, projectorPoint;

    cv::Point2f input_point(req.u, req.v);
    projectorPoint.push_back(input_point);
    cv::perspectiveTransform(projectorPoint, cameraPoint,
                             border_homography.inv());
    res.u_prime = cameraPoint[0].x;
    res.v_prime = cameraPoint[0].y;

    return true;
  }

  ros::NodeHandle* nh_;
  ros::ServiceServer projector_point_transform_service_,
      reverse_projector_point_transform_service_;
  cv::Matx33d border_homography, button_homography;
  std::vector<double> border_homography_array, button_homography_array;
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
  ros::init(argc, argv, "transform_projector_server");
  ros::NodeHandle nh;
  TransformProjectorPointServer server(&nh);

  ros::spin();

  return 0;
}