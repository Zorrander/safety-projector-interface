#ifndef PROJECTOR_AR_TRANSFORMS__CAMERA_TRANSFORM_NODE_HPP_
#define PROJECTOR_AR_TRANSFORMS__CAMERA_TRANSFORM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "tuni_whitegoods_msgs/srv/transform3_d_to_pixel.hpp"
#include "tuni_whitegoods_msgs/srv/transform_pixel_to3_d.hpp"

namespace tuni_whitegoods_transformations {

class CameraTransformNode : public rclcpp::Node {
public:
  explicit CameraTransformNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Service<tuni_whitegoods_msgs::srv::Transform3DToPixel>::SharedPtr service_3d_to_pixel_;
  rclcpp::Service<tuni_whitegoods_msgs::srv::TransformPixelTo3D>::SharedPtr service_pixel_to_3d_;

  double fx_, fy_, cx_, cy_;
  double k1_, k2_, k3_, p1_, p2_;

  bool handle3DToPixel(
    const std::shared_ptr<tuni_whitegoods_msgs::srv::Transform3DToPixel::Request> req,
    std::shared_ptr<tuni_whitegoods_msgs::srv::Transform3DToPixel::Response> res);

  bool handlePixelTo3D(
    const std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelTo3D::Request> req,
    std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelTo3D::Response> res);
};

}  // namespace tuni_whitegoods_transformations

#endif  // PROJECTOR_AR_TRANSFORMS__CAMERA_TRANSFORM_NODE_HPP_