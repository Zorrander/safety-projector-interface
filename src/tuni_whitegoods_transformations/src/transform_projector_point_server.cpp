#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "tuni_whitegoods_msgs/TransformPixelToProjection.h"


class TransformProjectorPointServer
{
public:
    TransformProjectorPointServer(ros::NodeHandle& nh)
        : nh_(nh)
    {
        projector_point_transform_service_ = nh_.advertiseService("transform_point_to_project", &TransformProjectorPointServer::transformProjectorPointCallback, this);
        homography = cv::Mat(3, 3, CV_32FC1);
    }

private:
    bool transformProjectorPointCallback(tuni_whitegoods_msgs::TransformPixelToProjection::Request &req, 
                                         tuni_whitegoods_msgs::TransformPixelToProjection::Response &res)
    {
        std::vector<cv::Point2f> cameraPoint, projectorPoint;

        cv::Point2f input_point(req.u, req.v);
        cameraPoint.push_back(input_point);
        cv::perspectiveTransform(cameraPoint, projectorPoint, homography);
        res.u_prime = projectorPoint[0].x;
        res.v_prime = projectorPoint[0].y;

        return true;
    }


    ros::NodeHandle nh_;
    ros::ServiceServer projector_point_transform_service_;
    cv::Mat homography;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_camera_point_server");
    ros::NodeHandle nh;
    TransformProjectorPointServer server(nh);

    ros::spin();

    return 0;
}