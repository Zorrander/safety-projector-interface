#ifndef DisplayArea_H
#define DisplayArea_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include "tuni_whitegoods_projector_interface/button.h"
#include "tuni_whitegoods_projector_interface/static_border.h"

class DisplayArea {
private:
	std::string name;
    cv::Rect camera_frame_area;

    std::vector<std::shared_ptr<StaticBorder>> borders_;


public:
    DisplayArea(std::string name);
    void add_border(StaticBorder border);
    void fetchButtons(std::vector<std::shared_ptr<Button>> buttons);
    void fetchBorders(std::vector<std::shared_ptr<StaticBorder>> borders);
    void checkForInteractions(std::string name, geometry_msgs::Point hand_position);
};

#endif