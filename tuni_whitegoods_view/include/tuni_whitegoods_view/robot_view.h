#ifndef RobotView_H
#define RobotView_H

#include "tuni_whitegoods_view/view.h"

class RobotView : public View {
 private:
  ros::NodeHandle nh_;
  ros::Subscriber marker_sub;
  ros::Publisher vis_pub;
  int marker_counter;
  std_msgs::ColorRGBA hand_color;

 public:
  RobotView(ros::NodeHandle* nh);

  void init(std::vector<std::shared_ptr<DisplayArea>> zones) override;
  void updateButtons(
      const std::vector<std::shared_ptr<Button>>& buttons) override;
  void updateBorders(
      const std::vector<std::shared_ptr<StaticBorder>>& borders) override;
  void updateHands(const std::vector<std::shared_ptr<Hand>>& hands) override;

  void createRvizMarker(std::vector<geometry_msgs::Point> points,
                        std_msgs::ColorRGBA color, int id = 1);
};

#endif