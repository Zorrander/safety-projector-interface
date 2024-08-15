#ifndef Hand_H
#define Hand_H

#include <string>
#include <geometry_msgs/Point.h>


class Hand {
private:
	std::string name;
	geometry_msgs::Point pixel_position,
						 projected_position,
						 robot_frame_position;

public:
    Hand(std::string name);
    void set_positon(geometry_msgs::Point pixel_position, geometry_msgs::Point projected_position, geometry_msgs::Point robot_frame_position);

};

#endif