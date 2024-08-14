#ifndef DisplayArea_H
#define DisplayArea_H

#include <string>
#include <opencv2/opencv.hpp>


class DisplayArea {
private:
	std::string name;
    cv::Rect camera_frame_area;

public:
    DisplayArea(std::string name);

};

#endif