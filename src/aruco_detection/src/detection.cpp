#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <unity_msgs/ArucoArray.h>
#include <unity_msgs/ArucoMarker.h>
#include <unity_msgs/HomographyMtx.h>
#include <opencv2/aruco.hpp>

using namespace std;
static const std::string OPENCV_WINDOW = "Image window";

class ArucoDetection
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher pub_markers;
    ros::Publisher pub_markers_warped;
    ros::Publisher pub_changes;
    cv_bridge::CvImagePtr cv_ptr;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
    std::vector<int> ids_warped;
    std::vector<std::vector<cv::Point2f>> corners_warped, rejectedCandidates_warped;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    unity_msgs::ArucoArray markers;
    unity_msgs::ArucoArray markers_warped;
    cv::Point2f vert_view_size;
    cv::Mat cv_hom;
    std::vector<float> prev_tlp;
    std::vector<float> prev_brp;
    

public:
    ArucoDetection():
    it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/rgb/image_raw", 1, &ArucoDetection::imageCb, this);
        //hom_sub = nh_.subscribe("/unity/table_homography", 1, &ArucoDetection::callbackHom,this);
        //pub_markers = nh_.advertise<unity_msgs::ArucoArray> ("/aruco_markers", 1);
        pub_markers_warped = nh_.advertise<unity_msgs::ArucoArray> ("/aruco_markers_warped", 1);
        pub_changes = nh_.advertise<std_msgs::Bool> ("/aruco_markers_warped/changes", 1);
        parameters = cv::aruco::DetectorParameters::create();
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        cv_hom = cv::Mat(3,3,CV_32FC1);
        prev_tlp.resize(0);
        prev_brp.resize(0);
        cv::namedWindow(OPENCV_WINDOW,cv::WINDOW_NORMAL);
    }

    ~ArucoDetection()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        fillHomography();
        cv::Mat image_cp;
        cv::Mat img_transformed(720,1280,CV_8UC3,cv::Scalar(0,0,0));
        cv_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr->image.copyTo(image_cp);
        //cout<<"test 1\n";
        cv::warpPerspective(cv_ptr->image,image_cp,cv_hom,image_cp.size());
        //cout<<"test 2\n";
        //cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);
        cv::aruco::detectMarkers(image_cp, dictionary, corners_warped, ids_warped);
       // cout<<"test 3\n";
        //cout<<ids_warped.size()<<"\n";
        //cout<<corners_warped.size()<<"\n";
        if(ids_warped.size() == 1)
        {
            markers_warped = prepareData(ids_warped,corners_warped);
            //cout<<"test 4\n";
            publishMarkers(markers_warped);
            //cout<<"test 5\n";
            detectChanges(ids_warped,corners_warped);
            //cout<<"test 6\n";
        }
        // else
        // {
        //     unity_msgs::ArucoArray tmp;
        //     tmp.array_markers.resize(0);
        //     publishMarkers(tmp);
        // }
        
            
        /*if (ids_warped.size() > 0)
        {
           //cv::aruco::drawDetectedMarkers(image_cp, corners_warped, ids_warped);
            std::cout<<"DETECTED !\n";
        }
        else
        {
            std::cout<<"NOT DETECTED \n";
        }*/
        //cv::imshow(OPENCV_WINDOW, image_cp);
        //cv::waitKey(1);
    }

    void detectChanges(std::vector<int> id, std::vector<std::vector<cv::Point2f>> cor)
    {
        std_msgs::Bool change;
        std::vector<float> v_l;
        std::vector<float> v_r;
        float diff_tl_x;
        float diff_tl_y;
        float diff_br_x;
        float diff_br_y;
        v_l.resize(0);
        v_r.resize(0);
        for(int i = 0; i < id.size();  i++)
        {
            v_l.push_back(std::abs(cor[id[i]][0].x - cor[id[i]][3].x));
            v_l.push_back(std::abs(cor[id[i]][0].y - cor[id[i]][3].y));
            v_r.push_back(std::abs(cor[id[i]][1].x - cor[id[i]][2].x));
            v_r.push_back(std::abs(cor[id[i]][1].y - cor[id[i]][2].y));
            float c_x = v_l[0] / v_r[0];
            float c_y = v_l[1] / v_r[1];
            float threshold = std::abs(c_x - c_y);
            if(prev_tlp.size() > 0)
            {
                diff_tl_x = std::abs(prev_tlp[0] - cor[id[i]][0].x);
                diff_tl_y = std::abs(prev_tlp[1] - cor[id[i]][0].y);
                diff_br_x = std::abs(prev_brp[0] - cor[id[i]][2].x);
                diff_br_y = std::abs(prev_brp[1] - cor[id[i]][2].y);
                if(diff_tl_x > 1 || diff_tl_y > 1 || diff_br_x > 1 || diff_br_y > 1)
                {
                    change.data = true;
                    prev_tlp.resize(0);
                    prev_brp.resize(0);
                    prev_tlp.push_back(cor[id[i]][0].x);
                    prev_tlp.push_back(cor[id[i]][0].y);
                    prev_brp.push_back(cor[id[i]][2].x);
                    prev_brp.push_back(cor[id[i]][2].y);
                }
                else
                {
                    change.data = false;
                }
            }
            else
            {
                prev_tlp.push_back(cor[id[i]][0].x);
                prev_tlp.push_back(cor[id[i]][0].y);
                prev_brp.push_back(cor[id[i]][2].x);
                prev_brp.push_back(cor[id[i]][2].y);
            }
        }
        pub_changes.publish(change);
    }

    void callbackHom(const unity_msgs::HomographyMtx::ConstPtr& msg)
    {
        double transform[3][3];
        cv_hom = cv::Mat(3,3,CV_32FC1);
        int k = 0;
        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                transform[i][j] = msg->matrix[k];
                cv_hom.at<float>(i,j) = msg->matrix[k];
                k++;
            }
        }
    //std::cout<<"print matrix : \n";
    //printTransform(transform);

    }

    unity_msgs::ArucoArray prepareData(std::vector<int> id, std::vector<std::vector<cv::Point2f>> cor)
    {
        unity_msgs::ArucoArray data;
        unity_msgs::ArucoMarker mark;
        data.array_markers.resize(0);
        mark.tl_corner.resize(0);
        mark.tr_corner.resize(0);
        mark.br_corner.resize(0);
        mark.bl_corner.resize(0);
        int size = id.size();
        for(int i = 0; i < size;  i++)
        {
            mark.id = id[i];
            mark.tl_corner.push_back(cor[id[i]][0].x);
            mark.tl_corner.push_back(cor[id[i]][0].y);
            mark.tr_corner.push_back(cor[id[i]][1].x);
            mark.tr_corner.push_back(cor[id[i]][1].y);
            mark.br_corner.push_back(cor[id[i]][2].x);
            mark.br_corner.push_back(cor[id[i]][2].y);
            mark.bl_corner.push_back(cor[id[i]][3].x);
            mark.bl_corner.push_back(cor[id[i]][3].y);
            mark.width_marker.push_back(cor[id[i]][1].x - cor[id[i]][0].x);
            mark.width_marker.push_back(cor[id[i]][1].y - cor[id[i]][0].y);
            mark.height_marker.push_back(cor[id[i]][3].x - cor[id[i]][0].x);
            mark.height_marker.push_back(cor[id[i]][3].y - cor[id[i]][0].y);
        }
        data.array_markers.push_back(mark);
        return data;
    }

    void publishMarkers(unity_msgs::ArucoArray data_warped)
    {
        //pub_markers.publish(data);
        pub_markers_warped.publish(data_warped);
    }

    void fillHomography()
    {
        cv_hom.at<float>(0,0) = 1.0173655;
        cv_hom.at<float>(0,1) = -7.42959127e-03;
        cv_hom.at<float>(0,2) = -1.19129080e+02;
        cv_hom.at<float>(1,0) = -1.10325402e-01;
        cv_hom.at<float>(1,1) = 9.26611194e-01;
        cv_hom.at<float>(1,2) = 8.39185172e+01;
        cv_hom.at<float>(2,0) = 3.32224481e-05;
        cv_hom.at<float>(2,1) = -2.24753766e-04;
        cv_hom.at<float>(2,2) = 1.00;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_detection");
    ArucoDetection detect;
    ros::spin();
    return 0;
}