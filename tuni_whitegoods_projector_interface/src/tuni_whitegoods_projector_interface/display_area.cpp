#include "tuni_whitegoods_projector_interface/display_area.h"

#include <integration/SafetyBorderViolation.h>
#include <integration/VirtualButtonEventArray.h>

DisplayArea::DisplayArea(ros::NodeHandle *nh, std::string name) : nh_(nh), name(name) {
  pub_border_violation = nh->advertise<integration::SafetyBorderViolation>("/execution/projector_interface/integration/topics/safety_border_violation",1);
  pub_button_event = nh_->advertise<integration::VirtualButtonEventArray> ("/execution/projector_interface/integration/topics/virtual_button_event_array", 1);
}

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
      if (border->checkForInteractions(name, cv_hand_position)){
          integration::SafetyBorderViolation msg_border;
          geometry_msgs::PolygonStamped initial_border;
          geometry_msgs::Pose target_location;

          msg_border.header.frame_id = "base";
          msg_border.request_id = border->getId();

          //initial_border.polygon.points.push_back();
          //initial_border.polygon.points.push_back();
          //initial_border.polygon.points.push_back();
          //initial_border.polygon.points.push_back();

          //target_location.position = ;

          //msg_border.initial_border = initial_border;
          //msg_border.target_location = target_location;
          pub_border_violation.publish(msg_border);
      }
    }
  }

  for (auto &button : buttons_) {
    integration::VirtualButtonEventArray events;
    if (button->checkForInteractions(name, cv_hand_position)){
      if (!button->isAlreadyPressed()){
        button->setAlreadyPressed(true);
        integration::VirtualButtonEvent msg_event;
        msg_event.virtual_button_id = button->getId();
        msg_event.event_type = msg_event.PRESSED;
        events.virtual_button_events.push_back(msg_event);
        pub_button_event.publish(events);
      }
    } else {
      if (button->isAlreadyPressed()){
        integration::VirtualButtonEvent msg_event;
        msg_event.virtual_button_id = button->getId();
        msg_event.event_type = msg_event.RELEASED;
        events.virtual_button_events.push_back(msg_event);
        pub_button_event.publish(events);

        button->setAlreadyPressed(false);
      }
    }
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
  for (auto button: buttons_)
  {
   buttons.push_back(button);
  }
}


void DisplayArea::addBorder(std::shared_ptr<StaticBorder> sb) {
  borders_.push_back(sb);
}

void DisplayArea::fetchBorders(
    std::vector<std::shared_ptr<StaticBorder>>& borders) {
  for (auto border: borders_)
  {
   borders.push_back(border);
  }
}
