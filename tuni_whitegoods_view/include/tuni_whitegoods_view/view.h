#ifndef VIEW_H
#define VIEW_H

#include <ros/ros.h>

#include <iostream>

#include "tuni_whitegoods_projector_interface/button.h"
#include "tuni_whitegoods_projector_interface/static_border.h"
#include "tuni_whitegoods_projector_interface/hand.h"

class View
{


public:
    View();
    virtual void init();
    virtual void updateButtons(std::vector<std::shared_ptr<Button>> buttons);
    virtual void updateBorders(std::vector<std::shared_ptr<StaticBorder>> borders);
    virtual void updateHands(std::vector<std::shared_ptr<Hand>> hands);
};

#endif