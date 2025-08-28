#ifndef PROJECTOR_AR_TRANSFORMS__TF_TRANSFORM_NODE_HPP_
#define PROJECTOR_AR_TRANSFORMS__TF_TRANSFORM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tuni_whitegoods_msgs/srv/transform_robot_camera_coordinates.hpp"

namespace tuni_whitegoods_transformations {

class TFTransformNode : public rclcpp::Node {
public:
  explicit TFTransformNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Service<tuni_whitegoods_msgs::srv::TransformRobotCameraCoordinates>::SharedPtr tf_srv_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool handleTransform(
    const std::shared_ptr<tuni_whitegoods_msgs::srv::TransformRobotCameraCoordinates::Request> req,
    std::shared_ptr<tuni_whitegoods_msgs::srv::TransformRobotCameraCoordinates::Response> res);
};

}  // namespace tuni_whitegoods_transformations

#endif