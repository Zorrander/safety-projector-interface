#include "tuni_whitegoods_transformations/tf_transform_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace tuni_whitegoods_transformations {

TFTransformNode::TFTransformNode(const rclcpp::NodeOptions & options)
: Node("tf_transform_node", options)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_srv_ = this->create_service<tuni_whitegoods_msgs::srv::TransformRobotCameraCoordinates>(
    "transform_world_coordinates_frame",
    std::bind(&TFTransformNode::handleTransform, this, std::placeholders::_1, std::placeholders::_2));
}

bool TFTransformNode::handleTransform(
  const std::shared_ptr<tuni_whitegoods_msgs::srv::TransformRobotCameraCoordinates::Request> req,
  std::shared_ptr<tuni_whitegoods_msgs::srv::TransformRobotCameraCoordinates::Response> res)
{
  try {
    geometry_msgs::msg::PoseStamped out =
      tf_buffer_->transform(req->in_point_stamped, req->target_frame, tf2::durationFromSec(0.1));
    res->out_point_stamped = out;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Failed to transform from '%s' to '%s': %s",
                req->in_point_stamped.header.frame_id.c_str(),
                req->target_frame.c_str(), ex.what());
  }
  return true;

}

}  // namespace tuni_whitegoods_transformations

RCLCPP_COMPONENTS_REGISTER_NODE(tuni_whitegoods_transformations::TFTransformNode)