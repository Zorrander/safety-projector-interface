#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include "tuni_whitegoods_msgs/TransformPixelToProjection.h"

class TransformProjectorPointServer
{
public:
    TransformProjectorPointServer(ros::NodeHandle* nh)
        : nh_(nh)
    {
        projector_point_transform_service_ = nh_->advertiseService("transform_point_to_project", &TransformProjectorPointServer::transformProjectorPointCallback, this);
        reverse_projector_point_transform_service_ = nh_->advertiseService("reverse_transform_point_to_project", &TransformProjectorPointServer::reverseTransformProjectorPointCallback, this);
        homography =  cv::Matx33d(1.70497912e+00,  4.42399276e-01, -7.37314713e+02,
                                  5.19667675e-02 , 2.20520788e+00, -4.49266091e+02,
                                  5.66463639e-05  ,5.47137996e-04,  1.00000000e+00);    }

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

    bool reverseTransformProjectorPointCallback(tuni_whitegoods_msgs::TransformPixelToProjection::Request &req, 
                                         tuni_whitegoods_msgs::TransformPixelToProjection::Response &res)
    {
        std::vector<cv::Point2f> cameraPoint, projectorPoint;

        cv::Point2f input_point(req.u, req.v);
        projectorPoint.push_back(input_point);
        cv::perspectiveTransform(projectorPoint, cameraPoint, homography.inv());
        res.u_prime = cameraPoint[0].x;
        res.v_prime = cameraPoint[0].y;

        return true;
    }

    ros::NodeHandle* nh_;
    ros::ServiceServer projector_point_transform_service_, reverse_projector_point_transform_service_;
    cv::Matx33d homography;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_projector_server");
    ros::NodeHandle nh;
    TransformProjectorPointServer server(&nh);

    ros::spin(); 

    return 0;
}