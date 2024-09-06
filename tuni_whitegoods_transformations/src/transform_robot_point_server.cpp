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

    }

private:
    bool transformWorldPointCallback(tuni_whitegoods_msgs::TransformRobotCameraCoordinates::Request &req, 
                                     tuni_whitegoods_msgs::TransformRobotCameraCoordinates::Response &res)
    {
        if (tfBuffer.canTransform(req.target_frame, req.in_point_stamped.header.frame_id, ros::Time(0))) {
            // The transform is available
            try {
                res.out_point_stamped = tfBuffer.transform(req.in_point_stamped, req.target_frame);
                /*
                ROS_INFO("After transformation: translation (%.2f, %.2f, %.2f), rotation (%.2f, %.2f, %.2f, %.2f)",
                         res.out_point_stamped.pose.position.x,
                         res.out_point_stamped.pose.position.y,
                         res.out_point_stamped.pose.position.z,
                         res.out_point_stamped.pose.orientation.x,
                         res.out_point_stamped.pose.orientation.y,
                         res.out_point_stamped.pose.orientation.z,
                         res.out_point_stamped.pose.orientation.w);*/
            } catch (tf2::TransformException &ex) {
                ROS_WARN("Transform Exception: %s", ex.what());
            }
        } else {
            ROS_WARN("Transform from %s to %s is not available", req.in_point_stamped.header.frame_id.c_str(), req.target_frame.c_str());
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