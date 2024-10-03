#include <ros/ros.h>

#include "tuni_whitegoods_msgs/Transform3DToPixel.h"
#include "tuni_whitegoods_msgs/TransformPixelTo3D.h"
#include "tuni_whitegoods_transformations/pinhole_camera.h"

/**
 * @brief      Camera points transformation.
 *
 * This class holds two service definitions. One to project points from 2D to
 * 3D space and the other to project points from 3D to 2D space.
 */
class TransformCameraPointServer {
 public:
  /**
   * @brief      Instantiates two service servers.
   *
   * Instantiates the transformation servers and loads the necessary intrinsic
   * parameters.
   *
   */
  TransformCameraPointServer(ros::NodeHandle *nh) : nh_(nh) {
    world_to_pixel_service_ = nh_->advertiseService(
        "transform_3D_to_pixel",
        &TransformCameraPointServer::transform3DToPixelCallback, this);
    pixel_to_3D_service_ = nh_->advertiseService(
        "transform_pixel_to_3D",
        &TransformCameraPointServer::transformPixelTo3DCallback, this);
    ros::param::get("fx", fx);
    ros::param::get("fy", fy);
    ros::param::get("cx", cx);
    ros::param::get("cy", cy);
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
  bool transform3DToPixelCallback(
      tuni_whitegoods_msgs::Transform3DToPixel::Request &req,
      tuni_whitegoods_msgs::Transform3DToPixel::Response &res) {
    res.u = fx * (req.x / req.z) + cx;
    res.v = fy * (req.y / req.z) + cy;

    return true;
  }

  /**
   * @brief      Service for transforming 2D points to 3D.
   *
   * This service takes a pixel value in a 2D image and projects
   * it onto its equivalent in 3D world coordinates.
   *
   * @param      req   Request object containing (u, v) pixel coordinates
   * @param      res   Response object containing (x, y, z) world coordinates.
   *
   * @return     true if the service call was successful, false otherwise.
   */
  bool transformPixelTo3DCallback(
      tuni_whitegoods_msgs::TransformPixelTo3D::Request &req,
      tuni_whitegoods_msgs::TransformPixelTo3D::Response &res) {
    res.x = (req.u - cx) * req.depth / fx;
    res.y = (req.v - cy) * req.depth / fy;
    res.z = req.depth;

    return true;
  }

  ros::NodeHandle *nh_;
  ros::ServiceServer pixel_to_3D_service_, world_to_pixel_service_;
  float fx, fy, cx, cy;
};

/**
 * @brief Main entry point for the Transform Camera Point Server.
 *
 * Initializes the ROS node, and starts the
 * TransformCameraPointServer. The server will handle service calls
 * related to transforming camera points in 3D space.
 *
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "transform_camera_point_server");
  ros::NodeHandle nh;
  TransformCameraPointServer server(&nh);

  ros::spin();

  return 0;
}