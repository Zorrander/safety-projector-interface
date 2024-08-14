#ifndef ProjectorInterfaceModel_H
#define ProjectorInterfaceModel_H

#include <string>
#include <vector>
#include <memory>

#include "tuni_whitegoods_projector_interface/display_area.h"



class ProjectorInterfaceController;

class ProjectorInterfaceModel {
private:
    std::vector<std::unique_ptr<DisplayArea>> zones;
    std::shared_ptr<ProjectorInterfaceController> observer_;

public:
    ProjectorInterfaceModel();
    void add_zone(std::string name);
    void attach(std::shared_ptr<ProjectorInterfaceController> observer);
    void detach();
    void notify();
};


#endif