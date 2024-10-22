#include "tuni_whitegoods_controller/projector_interface_controller.h"

#include <std_msgs/Empty.h>
#include <tuni_whitegoods_view/camera_view.h>
#include <tuni_whitegoods_view/project_view.h>
#include <tuni_whitegoods_view/robot_view.h>
#include <yaml-cpp/yaml.h>

int ProjectorInterfaceController::inboundPixel(int value, int min_val,
                                               int max_val) {
  return std::max(min_val, std::min(value, max_val));
}
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
  init_done = false;
  // Initialize model
  model_ = std::make_unique<ProjectorInterfaceModel>(nh_);

  detector = std::make_shared<ObjectDetector>();

  projector_view = std::make_shared<Projector>(nh_);
  camera_view = std::make_shared<CameraView>(nh_);
  robot_view = std::make_shared<RobotView>(nh_);

  // Initialize views
  views.push_back(projector_view);
  views.push_back(camera_view);
  views.push_back(robot_view);

  init_sub = nh_->subscribe("/odin/start", 1,
                            &ProjectorInterfaceController::initCallback, this);

  // Subscribe to model updates
  model_update_sub =
      nh_->subscribe("/odin/internal/model_changed", 10,
                     &ProjectorInterfaceController::modelUpdateCallback, this);

  depth_sub =
      nh_->subscribe("/depth_to_rgb/image_raw", 10,
                     &ProjectorInterfaceController::depthImageCallback, this);

  // Subscribe to hand detections
  hand_pose_sub =
      nh_->subscribe("/odin/internal/hand_detection", 10,
                     &ProjectorInterfaceController::handTrackerCallback, this);

  // Subscribe to moving table detections
  moving_table_pose_sub = nh_->subscribe(
      "/odin/projector_interface/moving_table", 10,
      &ProjectorInterfaceController::movingTableTrackerCallback, this);

  // Subscribe to commands coming from OpenFlow or custom scheduler
  service_borders = nh->advertiseService(
      "/execution/projector_interface/integration/services/"
      "list_static_border_status",
      &ProjectorInterfaceController::getBordersService, this);

  // Subscribe to robot coordinates if needed for dynamic border display
  /*
  TODO
  */

  ros::param::get("projector_resolution", projector_resolution);
  ros::param::get("camera_resolution", camera_resolution);

  ROS_INFO("ProjectorInterfaceController running");
}

void ProjectorInterfaceController::initCallback(
    const std_msgs::Empty::ConstPtr &msg) {
  init();
}

void ProjectorInterfaceController::init() {
  std::string display_areas_calibration_file;
  if (!nh_->getParam("display_areas_calibration_file",
                     display_areas_calibration_file)) {
    ROS_ERROR("display_areas calibration file is missing from configuration.");
  }

  YAML::Node display_areas_calibration =
      YAML::LoadFile(display_areas_calibration_file);

  ros::Duration(2.0).sleep();

  for (YAML::const_iterator it = display_areas_calibration.begin();
       it != display_areas_calibration.end(); ++it) {
    std::string area_name = it->first.as<std::string>();
    YAML::Node area_node = it->second;

    std::vector<geometry_msgs::Point> points;
    geometry_msgs::Point tl;
    geometry_msgs::Point tr;
    geometry_msgs::Point br;
    geometry_msgs::Point bl;

    tl.x = area_node["top_left"]["x"].as<double>();
    tl.y = area_node["top_left"]["y"].as<double>();
    tl.z = area_node["top_left"]["z"].as<double>();

    tr.x = area_node["top_right"]["x"].as<double>();
    tr.y = area_node["top_right"]["y"].as<double>();
    tr.z = area_node["top_right"]["z"].as<double>();

    br.x = area_node["bottom_right"]["x"].as<double>();
    br.y = area_node["bottom_right"]["y"].as<double>();
    br.z = area_node["bottom_right"]["z"].as<double>();

    bl.x = area_node["bottom_left"]["x"].as<double>();
    bl.y = area_node["bottom_left"]["y"].as<double>();
    bl.z = area_node["bottom_left"]["z"].as<double>();

    points.push_back(tl);
    points.push_back(tr);
    points.push_back(br);
    points.push_back(bl);

    std::shared_ptr<DisplayArea> display_area =
        std::make_shared<DisplayArea>(nh_, area_name);

    model_->add_zone(display_area, points);
  }

  ROS_INFO("Registered display areas");

  ros::Duration(1.0).sleep();

  // Add the camera field of view (for visualization only)

  std::vector<geometry_msgs::Point> camera_corners;
  geometry_msgs::Point camera_tl;
  geometry_msgs::Point camera_tr;
  geometry_msgs::Point camera_br;
  geometry_msgs::Point camera_bl;

  camera_tl.x = 0;
  camera_tl.y = 0;
  camera_tl.z = cv_depth.at<uint16_t>(camera_tl.y, camera_tl.x) / 1000.0;

  camera_tr.x = 0;
  camera_tr.y = camera_resolution[1];
  camera_tr.z = cv_depth.at<uint16_t>(camera_tr.y, camera_tr.x) / 1000.0;

  camera_br.x = camera_resolution[0];
  camera_br.y = camera_resolution[1];
  camera_br.z = cv_depth.at<uint16_t>(camera_br.y, camera_br.x) / 1000.0;

  camera_bl.x = camera_resolution[0];
  camera_bl.y = 0;
  camera_bl.z = cv_depth.at<uint16_t>(camera_bl.y, camera_bl.x) / 1000.0;

  camera_corners.push_back(camera_tl);
  camera_corners.push_back(camera_tr);
  camera_corners.push_back(camera_br);
  camera_corners.push_back(camera_bl);

  std::shared_ptr<DisplayArea> camera_area =
      std::make_shared<DisplayArea>(nh_, "camera");
  model_->add_zone(camera_area, camera_corners);

  ROS_INFO("Registered camera field of view");

  // Add the projection area (for visualization only)

  cv::Point projector_top_left(0, 0);
  cv::Point projector_top_right(0, projector_resolution[1]);
  cv::Point projector_bottom_right(projector_resolution[0],
                                   projector_resolution[1]);
  cv::Point projector_bottom_left(projector_resolution[0], 0);

  ROS_INFO("Found projector calibration");

  cv::Point transformed_projector_top_left =
      model_->fromProjector2Camera(projector_top_left);
  cv::Point transformed_projector_top_right =
      model_->fromProjector2Camera(projector_top_right);
  cv::Point transformed_projector_bottom_right =
      model_->fromProjector2Camera(projector_bottom_right);
  cv::Point transformed_projector_bottom_left =
      model_->fromProjector2Camera(projector_bottom_left);

  ROS_INFO("Processed projector calibration");

  std::vector<geometry_msgs::Point> projector_corners;
  geometry_msgs::Point projector_tl;
  geometry_msgs::Point projector_tr;
  geometry_msgs::Point projector_br;
  geometry_msgs::Point projector_bl;

  projector_tl.x =
      inboundPixel(transformed_projector_top_left.x, 0, camera_resolution[0]);
  projector_tl.y =
      inboundPixel(transformed_projector_top_left.y, 0, camera_resolution[1]);
  projector_tl.z =
      cv_depth.at<uint16_t>(projector_tl.y, projector_tl.x) / 1000.0;

  projector_tr.x =
      inboundPixel(transformed_projector_top_right.x, 0, camera_resolution[0]);
  projector_tr.y =
      inboundPixel(transformed_projector_top_right.y, 0, camera_resolution[1]);
  projector_tr.z =
      cv_depth.at<uint16_t>(projector_tr.y, projector_tr.x) / 1000.0;

  projector_br.x = inboundPixel(transformed_projector_bottom_right.x, 0,
                                camera_resolution[0]);
  projector_br.y = inboundPixel(transformed_projector_bottom_right.y, 0,
                                camera_resolution[1]);
  projector_br.z =
      cv_depth.at<uint16_t>(projector_br.y, projector_br.x) / 1000.0;

  projector_bl.x = inboundPixel(transformed_projector_bottom_left.x, 0,
                                camera_resolution[0]);
  projector_bl.y = inboundPixel(transformed_projector_bottom_left.y, 0,
                                camera_resolution[1]);
  projector_bl.z =
      cv_depth.at<uint16_t>(projector_bl.y, projector_bl.x) / 1000.0;

  projector_corners.push_back(projector_tl);
  projector_corners.push_back(projector_tr);
  projector_corners.push_back(projector_br);
  projector_corners.push_back(projector_bl);

  std::shared_ptr<DisplayArea> projector_area =
      std::make_shared<DisplayArea>(nh_, "projector");
  model_->add_zone(projector_area, projector_corners);

  ROS_INFO("Registered projection area");

  ros::Duration(1.0).sleep();

  for (auto &view : views) {
    view->init(model_->zones);
  }
  init_done = true;
}

void ProjectorInterfaceController::depthImageCallback(
    const sensor_msgs::ImageConstPtr &depth_msg) {
  // Check if the received image is empty
  if (depth_msg->data.empty()) {
    ROS_ERROR("Received an empty depth image!");
    return;
  }

  cv_bridge::CvImagePtr cv_bridge_depth;
  try {
    cv_bridge_depth = cv_bridge::toCvCopy(
        depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_depth = cv_bridge_depth->image;
  if (cv_depth.empty()) {
    ROS_ERROR("Converted depth image is empty!");
    return;
  }
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
  if (init_done) {
    for (auto &view : views) {
      view->updateButtons(model_->getButtons());
      view->updateBorders(model_->getBorders());
      view->updateHands(model_->getHands());
    }
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
