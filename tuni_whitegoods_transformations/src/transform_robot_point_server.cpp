#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "tuni_whitegoods_msgs/TransformRobotCameraCoordinates.h"

/**
 * @brief      World coordinates transformations.
 *
 * This class hold a service that transforms 3D points from one coordinates
 * frame to another.
 */
class TransformRobotPointServer {
 public:
  /**
   * @brief      Instantiate server.
   *
   * Initialize a tf Listener and start the service.
   *
   */
  TransformRobotPointServer(ros::NodeHandle* nh)
      : nh_(nh),
        tfBuffer(),
        tfListener(new tf2_ros::TransformListener(tfBuffer)) {
    world_coordinates_service_ = nh_->advertiseService(
        "transform_world_coordinates_frame",
        &TransformRobotPointServer::transformWorldPointCallback, this);
  }

 private:
  /**
   * @brief      Transforms 3D points from one coordinates frame to another.
   *
   * @param      req   Request contains a PoseStamped object and the target
   * coordinates frame to which the point should be converted.
   * @param      res   Response contains the converted PoseStamped.
   *
   * @return     true if the service call was successful, false otherwise.
   */
  bool transformWorldPointCallback(
      tuni_whitegoods_msgs::TransformRobotCameraCoordinates::Request& req,
      tuni_whitegoods_msgs::TransformRobotCameraCoordinates::Response& res) {
    if (tfBuffer.canTransform(req.target_frame,
                              req.in_point_stamped.header.frame_id,
                              ros::Time(0))) {
      try {
        res.out_point_stamped =
            tfBuffer.transform(req.in_point_stamped, req.target_frame);
      } catch (tf2::TransformException& ex) {
        ROS_WARN("Transform Exception: %s", ex.what());
      }
    } else {
      ROS_WARN("Transform from %s to %s is not available",
               req.in_point_stamped.header.frame_id.c_str(),
               req.target_frame.c_str());
    }

    return true;
  }

  ros::NodeHandle* nh_;
  ros::ServiceServer world_coordinates_service_;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener* tfListener;
};

/**
 * @brief Main entry point for the Transform Robot Point Server.
 *
 * Initializes the ROS node, and starts the
 * TransformRobotPointServer. The server will handle service calls
 * related to transforming points in 3D space.
 *
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "transform_robot_server");
  ros::NodeHandle nh;
  TransformRobotPointServer server(&nh);

  ros::spin();

  return 0;
}