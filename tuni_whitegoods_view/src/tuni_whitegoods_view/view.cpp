#include "tuni_whitegoods_view/view.h"

View::View() { ROS_INFO("New view"); }

void View::init() { ROS_INFO("No view to initalize"); }

void View::updateButtons(const std::vector<std::shared_ptr<Button>>& buttons) {
  ROS_INFO("No view to update");
}
void View::updateBorders(const std::vector<std::shared_ptr<StaticBorder>>& borders) {
  ROS_INFO("No view to update");
}
void View::updateHands(const std::vector<std::shared_ptr<Hand>>& hands) {
  ROS_INFO("No view to update");
}