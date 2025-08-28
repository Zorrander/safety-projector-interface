#include "tuni_whitegoods_transformations/projector_transform_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace tuni_whitegoods_transformations {

ProjectorTransformNode::ProjectorTransformNode(const rclcpp::NodeOptions & options)
: Node("projector_transform_node", options)
{
  auto border_array = this->declare_parameter<std::vector<double>>("border_homography");
  auto button_array = this->declare_parameter<std::vector<double>>("button_homography");
  auto res_long = this->declare_parameter<std::vector<long>>("projector_resolution", {1280, 720});
  projector_resolution_.assign(res_long.begin(), res_long.end());

  border_h_ = cv::Matx33d(border_array.data());
  button_h_ = cv::Matx33d(button_array.data());

  border_srv_ = this->create_service<tuni_whitegoods_msgs::srv::TransformPixelToProjection>(
    "transform_point_to_project",
    std::bind(&ProjectorTransformNode::handleBorder, this, std::placeholders::_1, std::placeholders::_2));

  button_srv_ = this->create_service<tuni_whitegoods_msgs::srv::TransformPixelToProjection>(
    "transform_point_to_smart_interface",
    std::bind(&ProjectorTransformNode::handleButton, this, std::placeholders::_1, std::placeholders::_2));

  reverse_srv_ = this->create_service<tuni_whitegoods_msgs::srv::TransformPixelToProjection>(
    "reverse_transform_point_to_project",
    std::bind(&ProjectorTransformNode::handleReverse, this, std::placeholders::_1, std::placeholders::_2));
}

int ProjectorTransformNode::clamp(int val, int max) {
  return std::max(0, std::min(val, max));
}

bool ProjectorTransformNode::handleBorder(
  const std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelToProjection::Request> req,
  std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelToProjection::Response> res)
{
  std::vector<cv::Point2f> in = { cv::Point2f(req->u, req->v) };
  std::vector<cv::Point2f> out;
  cv::perspectiveTransform(in, out, border_h_);

  int u = static_cast<int>(std::round(out[0].x));
  int v = static_cast<int>(std::round(out[0].y));

  res->u_prime = clamp(u, projector_resolution_[0] - 1);
  res->v_prime = clamp(v, projector_resolution_[1] - 1);
  return true;
}

bool ProjectorTransformNode::handleButton(
  const std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelToProjection::Request> req,
  std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelToProjection::Response> res)
{
  std::vector<cv::Point2f> in = { cv::Point2f(req->u, req->v) };
  std::vector<cv::Point2f> out;
  cv::perspectiveTransform(in, out, button_h_);

  int u = static_cast<int>(std::round(out[0].x));
  int v = static_cast<int>(std::round(out[0].y));

  res->u_prime = clamp(u, projector_resolution_[0] - 1);
  res->v_prime = clamp(v, projector_resolution_[1] - 1);
  return true;
}

bool ProjectorTransformNode::handleReverse(
  const std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelToProjection::Request> req,
  std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelToProjection::Response> res)
{
  std::vector<cv::Point2f> in = { cv::Point2f(req->u, req->v) };
  std::vector<cv::Point2f> out;
  cv::perspectiveTransform(in, out, border_h_.inv());

  int u = static_cast<int>(std::round(out[0].x));
  int v = static_cast<int>(std::round(out[0].y));

  res->u_prime = clamp(u, projector_resolution_[0] - 1);
  res->v_prime = clamp(v, projector_resolution_[1] - 1);
  return true;
}

} // namespace tuni_whitegoods_transformations

RCLCPP_COMPONENTS_REGISTER_NODE(tuni_whitegoods_transformations::ProjectorTransformNode)