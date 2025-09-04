#include "tuni_whitegoods_view/view.h"

View::View() { ROS_INFO("New view"); }

void View::init(std::vector<std::shared_ptr<DisplayArea>> zones) {
  ROS_INFO("No view to initalize");
}

void View::init() { ROS_INFO("No view to initalize"); }
void View::update(const std::vector<tuni_whitegoods_msgs::ZoneState>& zones) {
  ROS_INFO("No view to initalize");
}

void View::moveWindow() { ROS_INFO("No view to update"); }

void View::updateButtons(const std::vector<std::shared_ptr<Button>>& buttons,
                         std::shared_ptr<cv::Mat> layer) {
  ROS_INFO("No view to update");
}
void View::updateBorders(
    const std::vector<std::shared_ptr<StaticBorder>>& borders,
    std::shared_ptr<cv::Mat> layer) {
  ROS_INFO("No view to update");
}

void View::updateButtons(
    const std::vector<tuni_whitegoods_msgs::Button>& buttons,
    std::shared_ptr<cv::Mat> layer) {
  ROS_INFO("No view to update");
}
void View::updateBorders(
    const std::vector<tuni_whitegoods_msgs::StaticBorder>& borders,
    std::shared_ptr<cv::Mat> layer) {
  ROS_INFO("No view to update");
}
void View::updateHands(const std::vector<std::shared_ptr<Hand>>& hands) {
  ROS_INFO("No view to update");
}

void View::updateDisplayAreas(
    const std::vector<std::shared_ptr<DisplayArea>>& zones) {
  ROS_INFO("No view to update");
}