#ifndef VIEW_H
#define VIEW_H

#include <ros/ros.h>
#include <iostream>

class View
{


public:
    View();
    virtual void init();
    virtual void update();
};

#endif