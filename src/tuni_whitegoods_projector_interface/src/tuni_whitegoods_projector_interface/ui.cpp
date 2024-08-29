#include "button.h"
#include "instruction.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

class InterfaceUI {
public:
  InterfaceUI(const std::string &resource_id, const std::string &zone,
              const std::string &name, const std::string &description,
              const std::vector<Button> &buttons)
      : resource_id(resource_id), zone(zone), name(name),
        description(description), hidden(false), modified_interface(true) {
    if (!buttons.empty()) {
      list_buttons = buttons;
    }
    screen_size = cv::Size(1920, 1080);
    interface_img = cv::Mat::zeros(screen_size, CV_8UC3);
  }

  void add_button(const Button &button) {
    list_buttons.push_back(button);
    modified_interface = true;
  }

  void add_instruction(const Instruction &inst) {
    instructions.push_back(inst);
    modified_interface = true;
  }

  std::string get_zone() const { return zone; }

  const std::vector<Button> &get_list_buttons() const { return list_buttons; }

  bool get_hidden() const { return hidden; }

  cv::Mat draw() {
    if (modified_interface && !hidden) {
      for (auto &button : list_buttons) {
        button.draw_button(interface_img);
      }
      for (auto &instruction : instructions) {
        instruction.draw_instruction(interface_img);
      }
      modified_interface = false;
    }
    return interface_img;
  }

  void modify_button_color(const unity_msgs::ElementUI &button_msg) {
    for (auto &button : list_buttons) {
      if (button.get_id() == button_msg.id) {
        button.set_button_color(button_msg.button_color);
      }
    }
    modified_interface = true;
  }

  cv::Mat get_image_interface() const { return interface_img; }

  void print_buttons() const {
    for (const auto &button : list_buttons) {
      button.print_button();
    }
  }

private:
  std::string resource_id;
  std::string zone;
  std::string name;
  std::string description;
  std::vector<Button> list_buttons;
  std::vector<Instruction> instructions;
  bool hidden;
  bool modified_interface;
  cv::Size screen_size;
  cv::Mat interface_img;
};