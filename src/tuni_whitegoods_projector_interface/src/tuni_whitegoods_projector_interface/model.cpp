#include "tuni_whitegoods_projector_interface/model.h"


ProjectorInterfaceModel::ProjectorInterfaceModel()
{}

void ProjectorInterfaceModel::add_zone(std::string name){
	zones.push_back(std::make_unique<DisplayArea>(name));
}

void ProjectorInterfaceModel::attach(std::shared_ptr<ProjectorInterfaceController> observer){
	observer_ = observer;
}

void ProjectorInterfaceModel::detach(){
	observer_.reset();
}

void ProjectorInterfaceModel::notify(){
	observer_->notify();
}
