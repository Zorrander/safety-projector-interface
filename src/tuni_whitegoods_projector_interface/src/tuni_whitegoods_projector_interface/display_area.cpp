#include "tuni_whitegoods_projector_interface/display_area.h"

DisplayArea::DisplayArea(std::string name) : name(name) {}

void DisplayArea::checkForInteractions(std::string name,
                                       geometry_msgs::Point hand_position) {
  float distance;
  cv::Point cv_hand_position(static_cast<int>(hand_position.x),
                             static_cast<int>(hand_position.y));

  for (auto &border : borders_) {
    border->checkForInteractions(name, cv_hand_position);
  }
}

void DisplayArea::fetchButtons(
    std::vector<std::shared_ptr<Button>> buttons) {

}

void DisplayArea::fetchBorders(
    std::vector<std::shared_ptr<StaticBorder>> borders) {
  borders = borders_;
}
