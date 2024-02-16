#include <iostream>
#include <ros/ros.h>
//#include "use_class/NumberCounter.hpp"
#include "border/DynamicBorder.hpp"
#include <integration/SetPresetUIProjectionActionGoal.h>
#include <integration/SetPresetUIProjectionAction.h>
#include <integration/SetInstructionsProjectionAction.h>
#include <integration/SetInstructionsProjectionActionGoal.h>
#include <integration/SetSafetyBorderProjectionAction.h>
#include <integration/SetSafetyBorderProjectionActionGoal.h>
#include <integration/VirtualButtonReference.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>


using namespace integration;

typedef actionlib::SimpleActionClient<SetPresetUIProjectionAction> Client;
typedef actionlib::SimpleActionClient<SetInstructionsProjectionAction> Client_instruct;
typedef actionlib::SimpleActionClient<SetSafetyBorderProjectionAction> Client_static_border;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_case");
  ros::NodeHandle nh;
  Client client("/execution/projector_interface/integration/actions/set_preset_ui_projection", true);
  client.waitForServer();
  Client_instruct client_inst("execution/projector_interface/integration/actions/set_instructions_projection", true);
  client_inst.waitForServer();
  SetPresetUIProjectionGoal msg_goal;
  //ros::NodeHandle nh;
  //ros::Publisher pub_ui = nh.advertise<SetPresetUIProjectionActionGoal>("/execution/projector_interface/integration/actions/set_preset_ui_projection/goal",1);
  //SetPresetUIProjectionActionGoal msg_goal;
  VirtualButtonReference button_1;
  VirtualButtonReference button_2;
  VirtualButtonReference button_3;
  button_1.id = "20";
  button_1.zone = "table";
  button_1.name = "table";
  button_1.description = "button stop";
  button_1.text = "STOP";
  button_1.button_color.r = 1.0;
  button_1.button_color.g = 0.0;
  button_1.button_color.b = 0.0;
  button_1.button_color.a = 0.0;
  button_1.text_color.r = 1.0;
  button_1.text_color.g = 1.0;
  button_1.text_color.b = 1.0;
  button_1.text_color.a = 1.0;
  button_1.center.position.x = 150.0;
  button_1.center.position.y = 880.0;
  button_1.radius = 70.0;
  button_1.hidden = false;
  button_2.id = "40";
  button_2.zone = "table";
  button_2.name = "table";
  button_2.description = "button dead man";
  button_2.text = "GO";
  button_2.button_color.r = 0.0;
  button_2.button_color.g = 0.0;
  button_2.button_color.b = 1.0;
  button_2.button_color.a = 0.0;
  button_2.text_color.r = 1.0;
  button_2.text_color.g = 1.0;
  button_2.text_color.b = 1.0;
  button_2.text_color.a = 1.0;
  button_2.center.position.x = 840.0;
  button_2.center.position.y = 880.0;
  button_2.radius = 70.0;
  button_2.hidden = false;
  button_3.id = "30";
  button_3.zone = "table";
  button_3.name = "table";
  button_3.description = "button test";
  button_3.text = "TEST";
  button_3.button_color.r = 0.0;
  button_3.button_color.g = 1.0;
  button_3.button_color.b = 0.0;
  button_3.button_color.a = 0.0;
  button_3.text_color.r = 1.0;
  button_3.text_color.g = 1.0;
  button_3.text_color.b = 1.0;
  button_3.text_color.a = 1.0;
  button_3.center.position.x = 250.0;
  button_3.center.position.y = 100.0;
  button_3.radius = 70.0;
  button_3.hidden = false;
  msg_goal.request_id = "98097";
  msg_goal.zone = "table";
  msg_goal.resource_id = "01";
  msg_goal.name = "default ui";
  msg_goal.description = "default ui";
  msg_goal.hidden = false;
  msg_goal.virtual_button_references.clear();
  msg_goal.virtual_button_references.push_back(button_1);
  msg_goal.virtual_button_references.push_back(button_2);
  //msg_goal.virtual_button_references.push_back(button_3);


  /*client.sendGoal(msg_goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("interface sent!");
  printf("Current State: %s\n", client.getState().toString().c_str());

  ros::Duration(3.0).sleep();

  SetInstructionsProjectionGoal msg_inst;
  msg_inst.request_id = "286";
  msg_inst.zone = "table";
  msg_inst.target_location.position.x = 250.0;
  msg_inst.target_location.position.y = 500.0;
  msg_inst.title = "instructions";
  msg_inst.title_color.r = 0.0;
  msg_inst.title_color.g = 1.0;
  msg_inst.title_color.b = 0.0;
  msg_inst.title_color.a = 0.0;
  msg_inst.description = "press start to launch the next task";
  msg_inst.description_color.r = 0.0;
  msg_inst.description_color.g = 0.5;
  msg_inst.description_color.b = 1.0;
  msg_inst.description_color.a = 0.0;
  msg_inst.lifetime.fromNSec(0);
  client_inst.sendGoal(msg_inst);
  client_inst.waitForResult(ros::Duration(5.0));
  if (client_inst.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("instruction sent!");
  printf("Current State: %s\n", client_inst.getState().toString().c_str());*/

  Client_static_border client_border("/execution/projector_interface/integration/actions/set_safety_border_projection", true);
  client_border.waitForServer();
  SetSafetyBorderProjectionGoal msg;
  msg.request_id = "980";
  msg.zone = "shelf";
  geometry_msgs::PolygonStamped pl;
  msg.border.polygon.points.clear();
  msg.border.header.frame_id = "base";
  msg.border.header.stamp = ros::Time::now();
  geometry_msgs::Point32 p;
  p.x = 1.18; //1.73
  p.y = 1.0; //-0.08
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  p.x = 1.38; //0.5
  p.y = 1.3; 
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  msg.border_topic = "";
  msg.border_color.r = 0.0;
  msg.border_color.g = 1.0;
  msg.border_color.b = 0.0;
  msg.border_color.a = 0.0;
  msg.is_filled = false;
  msg.thickness = 1;
  msg.lifetime.fromNSec(0);
  msg.track_violations = true;
  client_border.sendGoal(msg);
  client_border.waitForResult(ros::Duration(5.0));
  if (client_border.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("static border sent!");
  printf("Current State: %s\n", client_border.getState().toString().c_str());

  ros::Duration(1.0).sleep();  
  msg.request_id = "180";
  msg.zone = "shelf";
  //geometry_msgs::PolygonStamped pl;
  msg.border.polygon.points.clear();
  msg.border.header.frame_id = "base";
  msg.border.header.stamp = ros::Time::now();
  //geometry_msgs::Point32 p;
  p.x = 1.42; //1.73
  p.y = 1.0; //-0.08
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  p.x = 1.62; //0.5
  p.y = 1.3; 
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  msg.border_topic = "";
  msg.border_color.r = 0.0;
  msg.border_color.g = 1.0;
  msg.border_color.b = 0.0;
  msg.border_color.a = 0.0;
  msg.is_filled = false;
  msg.thickness = 1;
  msg.lifetime.fromNSec(0);
  msg.track_violations = true;
  client_border.sendGoal(msg);
  client_border.waitForResult(ros::Duration(5.0));
  if (client_border.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("static border sent!");
  printf("Current State: %s\n", client_border.getState().toString().c_str());
  ros::Duration(1.0).sleep();
  
  msg.request_id = "280";
  msg.zone = "shelf";
  //geometry_msgs::PolygonStamped pl;
  msg.border.polygon.points.clear();
  msg.border.header.frame_id = "base";
  msg.border.header.stamp = ros::Time::now();
  //geometry_msgs::Point32 p;
  p.x = 1.64; //1.73
  p.y = 1.0; //-0.08
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  p.x = 1.84; //0.5
  p.y = 1.3; 
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  msg.border_topic = "";
  msg.border_color.r = 0.0;
  msg.border_color.g = 1.0;
  msg.border_color.b = 0.0;
  msg.border_color.a = 0.0;
  msg.is_filled = false;
  msg.thickness = 1;
  msg.lifetime.fromNSec(0);
  msg.track_violations = true;
  client_border.sendGoal(msg);
  client_border.waitForResult(ros::Duration(5.0));
  if (client_border.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("static border sent!");
  printf("Current State: %s\n", client_border.getState().toString().c_str());
  
  ros::Duration(1.0).sleep();
  msg.request_id = "380";
  msg.zone = "shelf";
  //geometry_msgs::PolygonStamped pl;
  msg.border.polygon.points.clear();
  msg.border.header.frame_id = "base";
  msg.border.header.stamp = ros::Time::now();
  //geometry_msgs::Point32 p;
  p.x = 1.88; //1.73
  p.y = 1.0; //-0.08
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  p.x = 2.08; //0.5
  p.y = 1.3; 
  p.z = -0.0;
  msg.border.polygon.points.push_back(p);
  msg.border_topic = "";
  msg.border_color.r = 0.0;
  msg.border_color.g = 1.0;
  msg.border_color.b = 0.0;
  msg.border_color.a = 0.0;
  msg.is_filled = false;
  msg.thickness = 1;
  msg.lifetime.fromNSec(0);
  msg.track_violations = true;
  client_border.sendGoal(msg);
  client_border.waitForResult(ros::Duration(5.0));
  if (client_border.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("static border sent!");
  printf("Current State: %s\n", client_border.getState().toString().c_str());
  
  ros::spin();



  
  return 0;
}