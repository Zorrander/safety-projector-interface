#include <ros/ros.h>

#include "tuni_whitegoods_msgs/Transform3DToPixel.h"
#include "tuni_whitegoods_msgs/TransformPixelTo3D.h"
#include "tuni_whitegoods_transformations/pinhole_camera.h"

class TransformCameraPointServer
{
public:
    TransformCameraPointServer(ros::NodeHandle* nh)
        : nh_(nh)
    {
        world_to_pixel_service_ = nh_->advertiseService("transform_3D_to_pixel", &TransformCameraPointServer::transform3DToPixelCallback, this);
        pixel_to_3D_service_ = nh_->advertiseService("transform_pixel_to_3D", &TransformCameraPointServer::transformPixelTo3DCallback, this);
        camera = PinholeCamera();
    }

private:
    bool transform3DToPixelCallback(tuni_whitegoods_msgs::Transform3DToPixel::Request &req, 
                                    tuni_whitegoods_msgs::Transform3DToPixel::Response &res)
    {
        res.u = camera.fx() * (req.x / req.z) + camera.cx() ; 
        res.v = camera.fy() * (req.y / req.z) + camera.cy() ;
        
        //ROS_INFO("3D Point in RGB camera coordinate system in pixel is : u = %d, v = %d", res.u , res.v);

        return true;
    }

    bool transformPixelTo3DCallback(tuni_whitegoods_msgs::TransformPixelTo3D::Request &req, 
                                    tuni_whitegoods_msgs::TransformPixelTo3D::Response &res)
    {
        res.x = (req.u - camera.cx()) * 1.4600766 / camera.fx();
        res.y = (req.v - camera.cy()) * 1.4600766 / camera.fy();
        res.z =  1.4600766;

        //ROS_INFO("3D Point in RGB camera coordinate system: X = %.3f, Y = %.3f, Z = %.3f", res.x , res.y , res.z);

        return true;
    }

    ros::NodeHandle* nh_;
    ros::ServiceServer pixel_to_3D_service_, world_to_pixel_service_;
    PinholeCamera camera;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_camera_point_server");
    ros::NodeHandle nh;
    TransformCameraPointServer server(&nh);

    ros::spin(); 

    return 0;
}