#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <string>

#include "tuni_whitegoods_msgs/Transform3DToPixel.h"
#include "tuni_whitegoods_msgs/TransformPixelTo3D.h"

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

    std::string calibration_file;
    if (!nh->getParam("camera_calibration_file", calibration_file)) {
      ROS_ERROR("Camera calibration file is missing from configuration.");
    }

    YAML::Node calibration = YAML::LoadFile(calibration_file);

    if (calibration["fx"]) {
      fx = calibration["fx"].as<double>();
    } else {
      ROS_ERROR("Missing camera calibration parameters.");
    }

    if (calibration["fy"]) {
      fy = calibration["fy"].as<double>();
    } else {
      ROS_ERROR("Missing camera calibration parameters.");
    }

    if (calibration["cx"]) {
      cx = calibration["cx"].as<double>();
    } else {
      ROS_ERROR("Missing camera calibration parameters.");
    }

    if (calibration["cy"]) {
      cy = calibration["cy"].as<double>();
    } else {
      ROS_ERROR("Missing camera calibration parameters.");
    }

    k1 = 0.5090121030807495;
    k2 = -2.8370401859283447;
    k3 = 0.00041892516310326755;
    k4 = -9.842617873800918e-05;
    k5 = 1.6451900005340576;
    k6 = 0.3761782646179199; 
    k7 = -2.630704641342163;
    k8 = 1.5565040111541748;

    k1= 0.0871772789718933;
    k2= -0.03671634009934126;
    k3= 0.0;
    p1= -0.001524745921348349 ;
    p2= 0.0002844683976229544;
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

  
    // Normalize the 3D point
    float x_d = req.x / req.z;
    float y_d = req.y / req.z;

    // Calculate the radial distance squared (r^2)
    float r2 = x_d * x_d + y_d * y_d;
    float r4 = r2 * r2;
    float r6 = r4 * r2;
    float r8 = r6 * r2;

    float radial_distortion = 1 + k1 * r2 + k2 * r4 + k3 * r6;
    float x_radial = x_d * radial_distortion;
    float y_radial = y_d * radial_distortion;

    // Step 4: Apply tangential distortion
    float x_tangential = 2 * p1 * x_d * y_d + p2 * (r2 + 2 * x_d * x_d);
    float y_tangential = p1 * (r2 + 2 * y_d * y_d) + 2 * p2 * x_d * y_d;

    float x_distorted = x_radial + x_tangential;
    float y_distorted = y_radial + y_tangential;

    // Step 5: Convert to pixel coordinates using the intrinsic matrix
    res.u = fx * x_distorted + cx;
    res.v = fy * y_distorted + cy;

    /*
    // Apply the rational polynomial distortion model
    // Coefficients D[0..7] are the rational polynomial distortion coefficients
    float distortion_numerator = 1 + k1 * r2 + k2 * r4 + k3 * r6 + k4 * r8;
    float distortion_denominator = 1 + k5 * r2 + k6 * r4 + k7 * r6 + k8 * r8;
    
    float distortion_factor = distortion_numerator / distortion_denominator;

    // Step 4: Apply distortion to normalized coordinates
    float x_distorted = x_d * distortion_factor;
    float y_distorted = y_d * distortion_factor;

    // Step 5: Convert to pixel coordinates using the intrinsic matrix
    res.u = fx * x_distorted + cx;
    res.v = fy * y_distorted + cy;
    */

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
  double fx;
  double fy;
  double cx;
  double cy;


  // Rational polynomial distortion coefficients
  double k1;
  double k2;
  double k3;
  double k4;
  double k5;
  double k6;
  double k7;
  double k8;
  double p1;
  double p2;
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