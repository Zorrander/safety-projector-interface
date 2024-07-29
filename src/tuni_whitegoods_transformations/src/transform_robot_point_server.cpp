#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "tuni_whitegoods_msgs/TransformRobotCameraCoordinates.h"


class TransformRobotPointServer
{
public:
    TransformRobotPointServer(ros::NodeHandle* nh)
        : nh_(nh), 
          tfBuffer(),  // Initialize tfBuffer
          tfListener(new tf2_ros::TransformListener(tfBuffer))  // Initialize tfListener with tfBuffer
    {
        world_coordinates_service_ = nh_->advertiseService("transform_world_coordinates_frame", &TransformRobotPointServer::transformWorldPointCallback, this);

        try {
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("base", "rgb_camera_link", ros::Time(0));
            ROS_INFO("Got transform: translation (%.2f, %.2f, %.2f), rotation (%.2f, %.2f, %.2f, %.2f)",
                     transformStamped.transform.translation.x,
                     transformStamped.transform.translation.y,
                     transformStamped.transform.translation.z,
                     transformStamped.transform.rotation.x,
                     transformStamped.transform.rotation.y,
                     transformStamped.transform.rotation.z,
                     transformStamped.transform.rotation.w);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF2 Transform Exception: %s", ex.what());
        }
    }

private:
    bool transformWorldPointCallback(tuni_whitegoods_msgs::TransformRobotCameraCoordinates::Request &req, 
                                     tuni_whitegoods_msgs::TransformRobotCameraCoordinates::Response &res)
    {
       try {
            res.out_point_stamped = tfBuffer.transform(req.in_point_stamped, req.target_frame);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF2 Transform Exception: %s", ex.what());    
        }
      
        return true;
    }

    ros::NodeHandle* nh_;
    ros::ServiceServer world_coordinates_service_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_robot_server");
    ros::NodeHandle nh;
    TransformRobotPointServer server(&nh);

    ros::spin();

    return 0;
}