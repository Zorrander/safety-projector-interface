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
        homography = cv::Matx33d(-2.15507712e+00,  1.91967042e-01,  2.86695078e+03, 
                                         5.92436261e-03, -2.16676604e+00,  1.75534894e+03, 
                                         1.69314309e-05,  2.45548501e-04,  1.00000000e+00);
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

    ros::waitForShutdown();

    return 0;
}