#ifndef VIEW_H
#define VIEW_H

#include <ros/ros.h>

#include <iostream>
#include <memory>
#include <vector>

#include "tuni_whitegoods_projector_interface/button.h"
#include "tuni_whitegoods_projector_interface/display_area.h"
#include "tuni_whitegoods_projector_interface/hand.h"
#include "tuni_whitegoods_projector_interface/static_border.h"

class View {
 public:
  View();
  virtual void init(std::vector<std::shared_ptr<DisplayArea>> zones);
  virtual void updateButtons(
      const std::vector<std::shared_ptr<Button>>& buttons);
  virtual void updateBorders(
      const std::vector<std::shared_ptr<StaticBorder>>& borders);
  virtual void updateHands(const std::vector<std::shared_ptr<Hand>>& hands);
  virtual void updateDisplayAreas(
      const std::vector<std::shared_ptr<DisplayArea>>& zones);
};

#endif