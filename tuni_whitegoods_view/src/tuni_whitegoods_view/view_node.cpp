#include <ros/ros.h>
#include <tuni_whitegoods_view/camera_view.h>
#include <tuni_whitegoods_view/project_view.h>
#include <tuni_whitegoods_view/robot_view.h>
#include <tuni_whitegoods_view/view.h>

#include "tuni_whitegoods_msgs/DisplayAreaStates.h"

class ViewsNode {
 public:
  ViewsNode(ros::NodeHandle* nh) : nh_(nh) {
    projector_view = std::make_shared<Projector>(nh_, 1);

    // second_projector_view = std::make_shared<Projector>(nh_, 2);
    // projector_view->window_name = "Projector 2";

    // camera_view = std::make_shared<CameraView>(nh_);

    // robot_view = std::make_shared<RobotView>(nh_);

    // Initialize views
    views.push_back(projector_view);
    // views.push_back(second_projector_view);
    // views.push_back(camera_view);
    // views.push_back(robot_view);

    model_state_srv =
        nh_->serviceClient<tuni_whitegoods_msgs::DisplayAreaStates>(
            "display_area_states");

    for (auto& view : views) {
      view->init();
    }
  }

  void update() {
    model_state_srv.call(model_state);
    for (auto& view : views) {
      view->update(model_state.response.zones);
    }
  }

 private:
  ros::NodeHandle* nh_;
  std::shared_ptr<View> projector_view, second_projector_view, camera_view,
      robot_view;
  std::vector<std::shared_ptr<View>> views;
  tuni_whitegoods_msgs::DisplayAreaStates model_state;
  ros::ServiceClient model_state_srv;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "views_node");
  ros::NodeHandle nh;
  ViewsNode viewsNode(&nh);

  ros::Rate loop_rate(2);
  ROS_INFO("VIEWS NODE RUNNING");
  while (ros::ok()) {
    // gui->update_imgui();
    viewsNode.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}