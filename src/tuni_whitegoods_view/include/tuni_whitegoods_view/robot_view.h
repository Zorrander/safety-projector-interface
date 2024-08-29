#ifndef RobotView_H
#define RobotView_H

#include "tuni_whitegoods_view/view.h"

#include <tuni_whitegoods_msgs/RobotViewElement.h>

class RobotView : public View {
private:
  ros::NodeHandle nh_;
  ros::Subscriber marker_sub;
  ros::Publisher vis_pub;

public:
  RobotView();

  void init() override;
  void updateButtons(std::vector<std::shared_ptr<Button>> buttons) override;
  void updateBorders(std::vector<std::shared_ptr<StaticBorder>> borders) override;
  void updateHands(std::vector<std::shared_ptr<Hand>> hands) override;

  void
  createRvizMarker(const tuni_whitegoods_msgs::RobotViewElement::ConstPtr &msg);
};

#endif