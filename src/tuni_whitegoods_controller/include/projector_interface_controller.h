#ifndef BookRobotStaticBorderServer_H
#define BookRobotStaticBorderServer_H


#include <ros/ros.h>


class ProjectorInterfaceController
{
protected:

    ros::NodeHandle* nh_;
    
    ProjectorInterfaceProjectorView projector_view_;
    ProjectorInterfaceCameraView camera_view_;
    ProjectorInterfaceRobotView robot_view_;

    ProjectorInterfaceModel model_;

    ros::Subscriber hand_pose_sub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;

    bool border_crossed; 
    bool button_pressed; 

public:
    void createBorderLayout(int rows, int cols, float sf_factor, bool adjacent, std_msgs::ColorRGBA status_booked, std_msgs::ColorRGBA status_free, std_msgs::ColorRGBA status_operator);
    void movingTableTrackerCallback(const tuni_whitegoods_msgs::HandsState& msg);
    void handTrackerCallback(const tuni_whitegoods_msgs::HandsState& msg);
    void handTrackingCallback(const unity_msgs::poiPCLConstPtr& msg);
    
    void addBorder(std::string r_id, std::string z, int pos_row, int pos_col, geometry_msgs::PolygonStamped bord, std::string b_topic, std_msgs::ColorRGBA b_color, bool filling, int thic, ros::Duration life, bool track); 
    void bookBorderRobot(std::string id);
    void bookBorderOperator(std::string id);
    void releaseRobotBorder(std::string id, int status);
    void releaseOperatorBorder(std::string id, int status);
    std::vector<std::string> getAdjacentBorders(int row, int col);
    bool getBordersService(integration::ListStaticBordersStatus::Request& req, integration::ListStaticBordersStatus::Response& res);
    void callback_button(const unity_msgs::ElementUI::ConstPtr &msg); 
    void process_button(double center_x, double center_y, const unity_msgs::ElementUI &msg);
    void callback_button_color(const unity_msgs::ElementUI::ConstPtr &msg);
};
#endif