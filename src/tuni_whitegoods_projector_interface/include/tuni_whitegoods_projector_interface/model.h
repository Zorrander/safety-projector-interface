#ifndef ProjectorInterfaceModel_H
#define ProjectorInterfaceModel_H

#include <string>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "tuni_whitegoods_projector_interface/hand.h"
#include "tuni_whitegoods_projector_interface/display_area.h"


class ProjectorInterfaceModel {
private:
	ros::NodeHandle* nh_;
    std::vector<std::unique_ptr<DisplayArea>> zones;

    std::unique_ptr<Hand> left_hand,
    					  right_hand;
    					  
    ros::Publisher pub_change_notification;

    ros::ServiceClient client_world_coordinates,
                       client_3D_to_pixel,
                       client_pixel_to_3D,
                       client_projector_point,
                       client_reverse_projector_point;
public:
    ProjectorInterfaceModel(ros::NodeHandle *nh);
    void add_zone(std::string name);
    void notify();
    void update_hand_pose(std::string name, geometry_msgs::Point position){
};


#endif