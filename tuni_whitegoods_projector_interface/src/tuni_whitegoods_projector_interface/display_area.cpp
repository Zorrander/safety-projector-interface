#include "tuni_whitegoods_projector_interface/display_area.h"

DisplayArea::DisplayArea(std::string name) : name(name) {}

void DisplayArea::create_border_layout(
    int rows, int cols, float sf_factor, bool adjacent,
    std_msgs::ColorRGBA status_booked, std_msgs::ColorRGBA status_free,
    std_msgs::ColorRGBA status_operator){
  border_layout = {
        rows,
        cols,
        sf_factor,
        adjacent,
        status_booked,
        status_free,
        status_operator
  };
}

void DisplayArea::checkForInteractions(std::string name,
                                       geometry_msgs::Point hand_position) {
  float distance;
  cv::Point cv_hand_position(static_cast<int>(hand_position.x),
                             static_cast<int>(hand_position.y));

  for (auto &border : borders_) {
    if (border->robot_booked || border->operator_booked){
      border->checkForInteractions(name, cv_hand_position);
    }
  }

  for (auto &button : buttons_) {
    button->checkForInteractions(name, cv_hand_position);
  }
}

void DisplayArea::resetInteractions() {
  for (auto &border : borders_) {
    border->resetInteractions();
  }

  for (auto &button : buttons_) {
    button->resetInteractions();
  }
}


void DisplayArea::addButton(std::shared_ptr<Button> btn) {
  buttons_.push_back(btn);
}

bool DisplayArea::change_button_color(std::string resource_id, std_msgs::ColorRGBA button_color){
  bool result = false;
  for (auto &button : buttons_) {
    if (button->get_name() == resource_id){
      button->set_button_color(button_color);
      result = true;
      break;
    }
  }
  return result;
}

bool DisplayArea::robot_book_border(std::string id){
  bool result = false;
  for (auto &border : borders_) {
    if (border->getId() == id){
      border->robot_book(border_layout.status_booked);
      if (border_layout.adjacent){
        for (auto &adj_border : borders_) {
          if(border->isAdjacent(adj_border)){
            adj_border->robot_book(border_layout.status_booked);
          }
        }
      }
      result = true;
      break;
    }
  }
  return result;
}

bool DisplayArea::robot_release_border(std::string id, int status){
  bool result = false;
  for (auto &border : borders_) {
    if (border->getId() == id){
      border->release(border_layout.status_free);
      border->resetInteractions();
      if (status == -1){
        for (auto &adj_border : borders_) {
          if(border->isAdjacent(adj_border)){
            adj_border->release(border_layout.status_free);
            adj_border->resetInteractions();
          }
        }      
      }
      result = true;
      break;
    }
  }
  return result;
}

bool DisplayArea::operator_book_border(std::string id){
  bool result = false;
  for (auto &border : borders_) {
    if (border->getId() == id){
      border->operator_book(border_layout.status_operator);
      result = true;
      break;
    }
  }
  return result;
}

bool DisplayArea::operator_release_border(std::string id, int status){
  bool result = false;
  for (auto &border : borders_) {
    if (border->getId() == id){
      border->release(border_layout.status_free);
      border->resetInteractions();
      if (status == -1){
        for (auto &adj_border : borders_) {
          if(border->isAdjacent(adj_border)){
            adj_border->release(border_layout.status_free);
            adj_border->resetInteractions();
          }
        }      
      }
      result = true;
      break;
    }
  }
  return result;
}

void DisplayArea::fetchButtons(
    std::vector<std::shared_ptr<Button>>& buttons) {
  buttons = buttons_;
}


void DisplayArea::addBorder(std::shared_ptr<StaticBorder> sb) {
  borders_.push_back(sb);
}

void DisplayArea::fetchBorders(
    std::vector<std::shared_ptr<StaticBorder>>& borders) {
  borders = borders_;
}
