#include "tuni_whitegoods_controller/projector_interface_controller.h"

#include <std_msgs/Empty.h>
#include <tuni_whitegoods_view/camera_view.h>
#include <tuni_whitegoods_view/project_view.h>
#include <tuni_whitegoods_view/robot_view.h>
#include <yaml-cpp/yaml.h>

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
  // Initialize model
  model_ = std::make_unique<ProjectorInterfaceModel>(nh_);

  detector = std::make_shared<ObjectDetector>(nh_);

  // Subscribe to commands coming from OpenFlow or custom scheduler
  service_borders = nh_->advertiseService(
      "/execution/projector_interface/integration/services/"
      "list_static_border_status",
      &ProjectorInterfaceController::getBordersService, this);

  projector_view = std::make_shared<Projector>(nh_, 1);
  projector_view->window_name = "Projector 1";

  // second_projector_view = std::make_shared<Projector>(nh_, 2);
  // projector_view->window_name = "Projector 2";

  camera_view = std::make_shared<CameraView>(nh_);

  robot_view = std::make_shared<RobotView>(nh_);

  // Initialize views
  views.push_back(projector_view);
  // views.push_back(second_projector_view);
  views.push_back(camera_view);
  // views.push_back(robot_view);

  init_sub = nh_->subscribe("/odin/start", 1,
                            &ProjectorInterfaceController::initCallback, this);

  // Subscribe to model updates
  model_update_sub =
      nh_->subscribe("/odin/internal/model_changed", 10,
                     &ProjectorInterfaceController::modelUpdateCallback, this);

  depth_sub =
      nh_->subscribe("/camera1/depth_to_rgb/image_raw", 10,
                     &ProjectorInterfaceController::depthImageCallback, this);

  // Subscribe to hand detections
  hand_pose_sub =
      nh_->subscribe("/odin/internal/hand_detection", 20,
                     &ProjectorInterfaceController::handTrackerCallback, this);

  transform_callback =
      nh->subscribe("/odin/projector_interface/moving_table/transform", 10,
                    &ProjectorInterfaceController::transformCallback, this);

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

  ROS_INFO("init");

  YAML::Node display_areas_calibration =
      YAML::LoadFile(display_areas_calibration_file);

  for (const auto &projector : display_areas_calibration) {
    int projector_id = std::stoi(projector.first.as<std::string>());

    for (const auto &area : projector.second) {
      std::string area_name = area.first.as<std::string>();
      YAML::Node area_node = area.second;

      std::vector<geometry_msgs::Point> points;
      geometry_msgs::Point tl;
      geometry_msgs::Point tr;
      geometry_msgs::Point br;
      geometry_msgs::Point bl;

      tl.x = area_node["top_left"]["x"].as<double>();
      tl.y = area_node["top_left"]["y"].as<double>();
      tl.z = area_node["top_left"]["z"].as<double>() / 1000.0;

      tr.x = area_node["top_right"]["x"].as<double>();
      tr.y = area_node["top_right"]["y"].as<double>();
      tr.z = area_node["top_right"]["z"].as<double>() / 1000.0;

      br.x = area_node["bottom_right"]["x"].as<double>();
      br.y = area_node["bottom_right"]["y"].as<double>();
      br.z = area_node["bottom_right"]["z"].as<double>() / 1000.0;

      bl.x = area_node["bottom_left"]["x"].as<double>();
      bl.y = area_node["bottom_left"]["y"].as<double>();
      bl.z = area_node["bottom_left"]["z"].as<double>() / 1000.0;

      points.push_back(tl);
      points.push_back(tr);
      points.push_back(br);
      points.push_back(bl);

      ROS_INFO("loaded");

      model_->add_zone(
          std::make_shared<DisplayArea>(nh_, area_name, projector_id), points);
    }
  }

  ROS_INFO("init 2");

  std::for_each(views.begin(), views.end(),
                [this](auto &view) { view->init(model_->getDisplayAreas()); });

  ROS_INFO("done");
  /*
  std::for_each(views.begin(), views.end(), [this](auto &view) {
    view->updateDisplayAreas(model_->getDisplayAreas());
  });*/
}

void ProjectorInterfaceController::transformCallback(
    const tuni_whitegoods_msgs::DynamicArea::ConstPtr &msg) {
  model_->updateMovingTable(*msg);

  std::for_each(views.begin(), views.end(), [this](auto &view) {
    view->updateDisplayAreas(model_->getDisplayAreas());
  });
}

void ProjectorInterfaceController::depthImageCallback(
    const sensor_msgs::ImageConstPtr &depth_msg) {
  try {
    cv_bridge_depth = cv_bridge::toCvCopy(
        depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

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
  model_->create_border_layout(rows, cols, sf_factor, adjacent, status_booked,
                               status_free, status_operator);
}

void ProjectorInterfaceController::handTrackerCallback(
    const tuni_whitegoods_msgs::HandsState &msg) {
  bool result = false;
  for (int i = 0; i < msg.name.size(); i++) {
    if (model_->updateHandPose(msg.name[i], msg.position[i])) {
      result = true;
    }
  }
}

void ProjectorInterfaceController::modelUpdateCallback(
    const std_msgs::Empty &msg) {
  std::for_each(views.begin(), views.end(), [this](auto &view) {
    view->updateDisplayAreas(model_->getDisplayAreas());
  });
}

void ProjectorInterfaceController::addInstructions(
    std::string zone, std::string title, std_msgs::ColorRGBA title_color) {
  model_->addInstructions(zone, title, title_color);
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

/* Book a robot border by its id
 *
 * @param[in]  id    The identifier
 */
void ProjectorInterfaceController::robot_book_border(std::string id) {
  ROS_INFO("BOOKING BORDER %s", id.c_str());
  model_->robot_book_border(id);
}

/** Book a border for the operator. It signals the operator that an object can
 * be picked by using a different color.
 *
 * @param[in]  id    The identifier
 */
void ProjectorInterfaceController::operator_book_border(std::string id) {
  ROS_INFO("bookBorderOperator");
  model_->operator_book_border(id);
}

/* release a border booked by the robot
 *
 * @param[in]  id      The identifier
 * @param[in]  status  The status
 */
void ProjectorInterfaceController::robot_release_border(std::string id,
                                                        int status) {
  // release booking and change color
  ROS_INFO("RELEASING BORDER %s -> %i", id.c_str(), status);
  model_->robot_release_border(id, status);
}

/* release a booking made by the operator
 *
 * @param[in]  id      The identifier
 * @param[in]  status  The status
 */
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
    ROS_INFO("Border %s", sbs.id.c_str());
    if (detector->scan(cv_depth(border->roi_rect), border->baseline) ||
        border->operator_booked) {
      sbs.status = 2;
      border->changeThickness(6);
    } else if (border->robot_booked) {
      sbs.status = 1;
      border->changeThickness(1);
    } else {
      sbs.status = 0;
      border->changeThickness(1);
    }

    ROS_INFO("border %s status: %d", sbs.id.c_str(), sbs.status);
    res.status_borders.push_back(sbs);
  }
  std::for_each(views.begin(), views.end(), [this](auto &view) {
    view->updateDisplayAreas(model_->getDisplayAreas());
  });

  ROS_INFO("Border status check complete.");
  return true;
}
