#include "tuni_whitegoods_transformations/camera_transform_node.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <rclcpp_components/register_node_macro.hpp>

namespace tuni_whitegoods_transformations {

CameraTransformNode::CameraTransformNode(const rclcpp::NodeOptions & options)
: Node("camera_transform_node", options) {
  // Declare and get parameters
  std::string calibration_file = this->declare_parameter("camera_calibration_file", "");

  if (calibration_file.empty()) {
    RCLCPP_ERROR(get_logger(), "camera_calibration_file param is missing.");
    return;
  }

  YAML::Node calib = YAML::LoadFile(calibration_file);
  fx_ = calib["fx"].as<double>();
  fy_ = calib["fy"].as<double>();
  cx_ = calib["cx"].as<double>();
  cy_ = calib["cy"].as<double>();

  k1_ = this->declare_parameter("k1", 0.0);
  k2_ = this->declare_parameter("k2", 0.0);
  k3_ = this->declare_parameter("k3", 0.0);
  p1_ = this->declare_parameter("p1", 0.0);
  p2_ = this->declare_parameter("p2", 0.0);

  service_3d_to_pixel_ = this->create_service<tuni_whitegoods_msgs::srv::Transform3DToPixel>(
    "transform_3D_to_pixel",
    std::bind(&CameraTransformNode::handle3DToPixel, this, std::placeholders::_1, std::placeholders::_2));

  service_pixel_to_3d_ = this->create_service<tuni_whitegoods_msgs::srv::TransformPixelTo3D>(
    "transform_pixel_to_3D",
    std::bind(&CameraTransformNode::handlePixelTo3D, this, std::placeholders::_1, std::placeholders::_2));
}

bool CameraTransformNode::handle3DToPixel(
  const std::shared_ptr<tuni_whitegoods_msgs::srv::Transform3DToPixel::Request> req,
  std::shared_ptr<tuni_whitegoods_msgs::srv::Transform3DToPixel::Response> res)
{
  double x = req->x / req->z;
  double y = req->y / req->z;
  double r2 = x*x + y*y;
  double r4 = r2*r2, r6 = r4*r2;

  // Radial distortion (polynomial)
  double radial = 1.0 + k1_*r2 + k2_*r4 + k3_*r6;

  double x_radial = x * radial;
  double y_radial = y * radial;

  // Tangential distortion (Brown–Conrady model)
  double x_tangential = 2.0*p1_*x*y + p2_*(r2 + 2.0*x*x);
  double y_tangential = p1_*(r2 + 2.0*y*y) + 2.0*p2_*x*y;

  double x_distorted = x_radial + x_tangential;
  double y_distorted = y_radial + y_tangential;

  res->u = fx_*x_distorted + cx_;
  res->v = fy_*y_distorted + cy_;
  return true;
}

bool CameraTransformNode::handlePixelTo3D(
  const std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelTo3D::Request> req,
  std::shared_ptr<tuni_whitegoods_msgs::srv::TransformPixelTo3D::Response> res)
{
  res->x = (req->u - cx_) * req->depth / fx_;
  res->y = (req->v - cy_) * req->depth / fy_;
  res->z = req->depth;
  return true;
}

} // namespace tuni_whitegoods_transformations

RCLCPP_COMPONENTS_REGISTER_NODE(tuni_whitegoods_transformations::CameraTransformNode)