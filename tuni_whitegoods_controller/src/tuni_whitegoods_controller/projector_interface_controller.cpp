#include "tuni_whitegoods_controller/projector_interface_controller.h"

#include <tuni_whitegoods_view/camera_view.h>
#include <tuni_whitegoods_view/project_view.h>
#include <tuni_whitegoods_view/robot_view.h>

/**
 * @brief      Controller class for the %projector interface.
 *
 * - receives perception input (hand) and update model
 * - track interactions
 * - update view (projector, camera and robot)
 *
 */
ProjectorInterfaceController::ProjectorInterfaceController(ros::NodeHandle *nh)
    : nh_(nh) {
  // Subscribe to hand detections
  hand_pose_sub =
      nh_->subscribe("/odin/internal/hand_detection", 10,
                     &ProjectorInterfaceController::handTrackerCallback, this);
  // Subscribe to moving table detections
  moving_table_pose_sub = nh_->subscribe(
      "/odin/projector_interface/moving_table", 10,
      &ProjectorInterfaceController::movingTableTrackerCallback, this);

  // Subscribe to robot coordinates if needed for dynamic border display
  /*base_link.subscribe(nh, "/coords/base_link", 1);
  link_4.subscribe(nh, "/coords/link_4", 1);
  link_5.subscribe(nh, "/coords/link_5", 1);
  tool_0.subscribe(nh, "/coords/tool0", 1);
  flange.subscribe(nh, "/coords/flange", 1);
  sync.reset(new Sync(MySyncPolicy(10), base_link, link_4, link_5, tool_0,
  flange)); sync->registerCallback(boost::bind(&DynamicBorder::callbackJoints3D,
  this, _1, _2, _3, _4, _5));
  */
  // Subscribe to commands coming from OpenFlow or custom scheduler
  service_borders = nh->advertiseService(
      "/execution/projector_interface/integration/services/"
      "list_static_border_status",
      &ProjectorInterfaceController::getBordersService, this);

  // Initialize model
  model_ = std::make_unique<ProjectorInterfaceModel>(nh_);
  model_->add_zone("shelf");
  model_->add_zone("table");
  // Subscribe to model updates
  model_update_sub =
      nh_->subscribe("/odin/internal/model_changed", 10,
                     &ProjectorInterfaceController::modelUpdateCallback, this);

  depth_sub =
      nh_->subscribe("/depth_to_rgb/image_raw", 20,
                     &ProjectorInterfaceController::depthImageCallback, this);

  detector = std::make_shared<ObjectDetector>();

  projector_view = std::make_shared<Projector>(nh_);
  camera_view = std::make_shared<CameraView>(nh_);
  robot_view = std::make_shared<RobotView>(nh_);

  // Initialize views
  views.push_back(projector_view);
  views.push_back(camera_view);
  views.push_back(robot_view);

  ros::Duration(3.0).sleep();

  for (auto &view : views) {
    view->init();
  }

  ROS_INFO("ProjectorInterfaceController running");
}

void ProjectorInterfaceController::depthImageCallback(
    const sensor_msgs::ImageConstPtr &depth_msg) {
  cv_bridge::CvImagePtr cv_bridge_depth =
      cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  cv_depth = cv_bridge_depth->image;
}

/**
 * @brief      Creates a border layout.
 *
 * @param[in]  rows             The rows
 * @param[in]  cols             The cols
 * @param[in]  sf_factor        The sf factor
 * @param[in]  adjacent         The adjacent
 * @param[in]  status_booked    The status booked
 * @param[in]  status_free      The status free
 * @param[in]  status_operator  The status operator
 */
void ProjectorInterfaceController::createBorderLayout(
    int rows, int cols, float sf_factor, bool adjacent,
    std_msgs::ColorRGBA status_booked, std_msgs::ColorRGBA status_free,
    std_msgs::ColorRGBA status_operator) {
  ROS_INFO("createBorderLayout");
  model_->create_border_layout(rows, cols, sf_factor, adjacent, status_booked,
                               status_free, status_operator);
}

void ProjectorInterfaceController::movingTableTrackerCallback(
    const tuni_whitegoods_msgs::DynamicArea &msg) {
  ROS_INFO("movingTableTrackerCallback");
}

void ProjectorInterfaceController::handTrackerCallback(
    const tuni_whitegoods_msgs::HandsState &msg) {
  for (int i = 0; i < msg.name.size(); i++) {
    geometry_msgs::Point position = msg.position[i];
    model_->updateHandPose(msg.name[i], position);
  }
}

void ProjectorInterfaceController::modelUpdateCallback(
    const std_msgs::Empty &msg) {
  for (auto &view : views) {
    view->updateButtons(model_->getButtons());
    view->updateBorders(model_->getBorders());
    view->updateHands(model_->getHands());
  }
}

/**
 * @brief      Adds a button.
 *
 * @param[in]  request_id    The request identifier
 * @param[in]  zone          The zone
 * @param[in]  name          The name
 * @param[in]  text          The text
 * @param[in]  button_color  The button color
 * @param[in]  text_color    The text color
 * @param[in]  center        The center
 * @param[in]  radius        The radius
 */
void ProjectorInterfaceController::addButton(
    std::string request_id, std::string zone, std::string name,
    std::string text, std_msgs::ColorRGBA button_color,
    std_msgs::ColorRGBA text_color, geometry_msgs::Pose center, float radius) {
  model_->addButton(request_id, zone, name, text, button_color, text_color,
                    center, radius);
}

/**
 * @brief      Changes color for a button already projected.
 *
 * @param[in]  resource_id   The resource identifier
 * @param[in]  button_color  The new button color
 */
void ProjectorInterfaceController::change_button_color(
    std::string resource_id, std_msgs::ColorRGBA button_color) {
  model_->change_button_color(resource_id, button_color);
}

/**
 * @brief      Adds a static border.
 *
 * @param[in]  r_id     The r identifier
 * @param[in]  z        { parameter_description }
 * @param[in]  pos_row  The position row
 * @param[in]  pos_col  The position col
 * @param[in]  bord     The bord
 * @param[in]  b_topic  The b topic
 * @param[in]  b_color  The b color
 * @param[in]  filling  The filling
 * @param[in]  thic     The thic
 * @param[in]  life     The life
 * @param[in]  track    The track
 */
void ProjectorInterfaceController::addStaticBorder(
    std::string r_id, std::string z, int pos_row, int pos_col,
    geometry_msgs::PolygonStamped bord, std::string b_topic,
    std_msgs::ColorRGBA b_color, bool filling, int thic, ros::Duration life,
    bool track) {
  model_->addStaticBorder(cv_depth, r_id, z, pos_row, pos_col, bord, b_topic,
                          b_color, filling, thic, life, track);
}

/**
 * @brief      Adds a dynamic border.
 *
 * @param[in]  r_id     The r identifier
 * @param[in]  z        { parameter_description }
 * @param[in]  b_topic  The b topic
 * @param[in]  b_color  The b color
 * @param[in]  filling  The filling
 * @param[in]  thic     The thic
 * @param[in]  life     The life
 * @param[in]  track    The track
 */
void ProjectorInterfaceController::addDynamicBorder(
    std::string r_id, std::string z, std::string b_topic,
    std_msgs::ColorRGBA b_color, bool filling, int thic, ros::Duration life,
    bool track) {}

// Book a robot border by its id
//
// @param[in]  id    The identifier
//
void ProjectorInterfaceController::robot_book_border(std::string id) {
  ROS_INFO("BOOKING BORDER %s", id.c_str());
  model_->robot_book_border(id);
}

// Book a border for the operator. It signals the operator that an object can be
// picked by using a different color.
//
// @param[in]  id    The identifier
//
void ProjectorInterfaceController::operator_book_border(std::string id) {
  ROS_INFO("bookBorderOperator");
  model_->operator_book_border(id);
}

// release a border booked by the robot
//
// @param[in]  id      The identifier
// @param[in]  status  The status
//
void ProjectorInterfaceController::robot_release_border(std::string id,
                                                        int status) {
  // release booking and change color
  ROS_INFO("RELEASING BORDER %s -> %i", id.c_str(), status);
  model_->robot_release_border(id, status);
}

// release a booking made by the operator
//
// @param[in]  id      The identifier
// @param[in]  status  The status
//
void ProjectorInterfaceController::operator_release_border(std::string id,
                                                           int status) {
  ROS_INFO("releaseOperatorBorder");
  model_->operator_release_border(id, status);
}

/**
 * @brief      Service for getting the status of the borders being projected.
 *
 * @param      req   Request empty
 * @param      res   Response contains the status of each border
 *
 * @return     true if the service call was successful, false otherwise.
 */
bool ProjectorInterfaceController::getBordersService(
    integration::ListStaticBordersStatus::Request &req,
    integration::ListStaticBordersStatus::Response &res) {
  ROS_INFO("Checking border status...");

  for (auto &border : model_->getBorders()) {
    integration::StaticBorderStatus sbs;
    sbs.id = border->getId();

    if (detector->scan(cv_depth(border->roi_rect), border->baseline) ||
        border->operator_booked) {
      sbs.status = 2;
    } else if (border->robot_booked) {
      sbs.status = 1;
    } else {
      sbs.status = 0;
    }

    ROS_INFO("border %s status: %d", sbs.id.c_str(), sbs.status);
    res.status_borders.push_back(sbs);
  }
  integration::StaticBorderStatus sbs;
  sbs.id = "border_four";
  sbs.status = 1;
  res.status_borders.push_back(sbs);
  ROS_INFO("border %s status: %d", sbs.id.c_str(), sbs.status);
  ROS_INFO("Border status check complete.");
  return true;
}
