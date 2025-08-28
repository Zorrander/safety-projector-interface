#ifndef PROJECTOR_AR_TRANSFORMS__PROJECTOR_TRANSFORM_NODE_HPP_
#define PROJECTOR_AR_TRANSFORMS__PROJECTOR_TRANSFORM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "tuni_whitegoods_msgs/srv/transform_pixel_to_projection.hpp"

namespace tuni_whitegoods_transformations {

class ProjectorTransformNode : public rclcpp::Node {
public:
  explicit ProjectorTransformNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Service<tuni_whitegoods_msgs::srv::TransformPixelToProjection>::SharedPtr border_srv_;
  rclcpp::Service<tuni_whitegoods_msgs::srv::TransformPixelToProjection>::SharedPtr button_srv_;
  rclcpp::Service<tuni_whitegoods_msgs::srv::TransformPixelToProjection>::SharedPtr reverse_srv_;

  cv::Matx33d border_h_, button_h_;
  std::vector<int> projector_resolution_;

  int clamp(int val, int max);
  bool handleBorder(
    const std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelToProjection::Request> req,
    std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelToProjection::Response> res);

  bool handleButton(
    const std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelToProjection::Request> req,
    std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelToProjection::Response> res);

  bool handleReverse(
    const std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelToProjection::Request> req,
    std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelToProjection::Response> res);
};

}  // namespace tuni_whitegoods_transformations

#endif