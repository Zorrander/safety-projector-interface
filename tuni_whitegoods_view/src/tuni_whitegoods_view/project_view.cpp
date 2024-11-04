#include "tuni_whitegoods_view/project_view.h"

#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cstdlib>
#include <thread>

#include "tf2/utils.h"

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

Projector::Projector(ros::NodeHandle *nh)
    : client_border(
          "/execution/projector_interface/integration/actions/"
          "set_safety_border_projection",
          true),
      ac("/execution/projector_interface/integration/actions/"
         "set_layout_static_borders",
         true),
      client_book_border_robot(
          "execution/projector_interface/integration/actions/"
          "book_robot_static_border",
          true),
      client_release_border_robot(
          "execution/projector_interface/integration/actions/"
          "release_robot_static_border",
          true),
      client_book_border_human(
          "execution/projector_interface/integration/actions/"
          "book_human_static_border",
          true),
      client_release_border_human(
          "execution/projector_interface/integration/actions/"
          "release_human_static_border",
          true),
      project_client(
          "/execution/projector_interface/integration/actions/"
          "set_virtual_buttons_projection",
          true),
      nh_(nh) {
  if (!ros::param::get("shiftX", shift)) {
    shift = 0;  // Default value
    ROS_WARN("Parameter 'shiftX' not found, using default value 0.");
  }
  ros::param::get("projector_resolution", projector_resolution);
  scan = false;
  tf = false;
  client_detection = nh->serviceClient<integration::ListStaticBordersStatus>(
      "/execution/projector_interface/integration/services/"
      "list_static_border_status");

  hand_detection_sub = nh_->subscribe("/odin/internal/hand_detection", 5,
                                      &Projector::handDetectionCallback, this);

  table_detection_sub =
      nh_->subscribe("/odin/projector_interface/moving_table/transform", 5,
                     &Projector::tableDetectionCallback, this);

  smart_interface_pub = nh->advertise<std_msgs::Empty>("/odin/start", 1);

  threshold_pub =
      nh->advertise<std_msgs::Int32>("/odin/object_detection/set_threshold", 1);
  non_zero_threshold_pub = nh->advertise<std_msgs::Int32>(
      "/odin/object_detection/set_non_zero_threshold", 1);
  noise_recuction_pub = nh->advertise<std_msgs::Int32>(
      "/odin/object_detection/set_noise_reduction", 1);

  layers["background"] = {
      std::make_shared<cv::Mat>(cv::Mat::zeros(
          projector_resolution[1], projector_resolution[0], CV_8UC3)),
      true};

  cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
  cv::moveWindow(OPENCV_WINDOW, shift, 0);
  cv::setWindowProperty(OPENCV_WINDOW, cv::WND_PROP_FULLSCREEN,
                        cv::WINDOW_FULLSCREEN);

  initializeGLFWandOpenGL();

  window = glfwCreateWindow(2300, 1300, "Layer Manager", nullptr, nullptr);
  glfwMakeContextCurrent(window);
  glewInit();

  initializeImGui(window);

  hand_detection_counter = 0;
  last_msg_time_ = ros::Time(0);
  interval_in_seconds_ = 0.0f;

  table_detection_counter = 0;
  last_table_msg_time_ = ros::Time(0);
  table_interval_in_seconds_ = 0.0f;

  ROS_INFO("ProjectorView running");
}

Projector::~Projector() { cv::destroyWindow(OPENCV_WINDOW); }

void Projector::init(std::vector<std::shared_ptr<DisplayArea>> zones) {
  display_areas = zones;
  combined = layers["background"].mat->clone();
  for (auto &zone : zones) {
    if (!(zone->name == "projector" || zone->name == "camera")) {
      layers[zone->name] = {
          std::make_shared<cv::Mat>(cv::Mat::zeros(
              projector_resolution[1], projector_resolution[0], CV_8UC3)),
          true};
    }
  }
}

void Projector::initializeGLFWandOpenGL() {
  if (!glfwInit()) {
    ROS_ERROR("Failed to initialize GLFW!");
    exit(EXIT_FAILURE);
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
}

void Projector::initializeImGui(GLFWwindow *window) {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330");
  ImGui::StyleColorsDark();
}

void Projector::cleanupImGui() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
}

GLuint Projector::cvMatToTexture(const cv::Mat &mat) {
  GLuint textureID;
  glGenTextures(1, &textureID);
  glBindTexture(GL_TEXTURE_2D, textureID);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  cv::Mat rgb;
  if (mat.channels() == 3)
    cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
  else
    rgb = mat;

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rgb.cols, rgb.rows, 0, GL_RGB,
               GL_UNSIGNED_BYTE, rgb.data);
  glBindTexture(GL_TEXTURE_2D, 0);

  return textureID;
}

void Projector::show_debug_borders() {
  ImGui::Begin("Borders");
  if (ImGui::BeginTabBar("BorderTabBar")) {
    for (auto &zone : display_areas) {
      std::vector<std::shared_ptr<StaticBorder>> borders;
      zone->fetchBorders(borders);
      for (auto &border : borders) {
        if (ImGui::BeginTabItem(border->getId().c_str())) {
          ImGui::Text(
              "Position in world coordinates -> top left corner: (x: %f, y: "
              "%f, z: %f), bottom right corner: (x: %f, y: %f, z: %f)",
              border->topLeftCornerPt.x, border->topLeftCornerPt.y,
              border->topLeftCornerPt.z, border->bottomRightCornerPt.x,
              border->bottomRightCornerPt.y, border->bottomRightCornerPt.z);
          ImGui::Text(
              "Position in camera view -> top left corner: (%d, %d), bottom "
              "right corner : (%d, %d)",
              border->top_left_cam_point.x, border->top_left_cam_point.y,
              border->bottom_right_cam_point.x,
              border->bottom_right_cam_point.y);
          ImGui::Text(
              "Position in projector view -> top left corner: (%d, %d), bottom "
              "right corner : (%d, %d)",
              border->top_left_proj_point.x, border->top_left_proj_point.y,
              border->bottom_right_proj_point.x,
              border->bottom_right_proj_point.y);

          static bool checkboxValue = false;
          ImGui::Checkbox("Release all", &checkboxValue);

          if (ImGui::Button("Book (Robot)")) {
            integration::BookRobotStaticBorderGoal goal;
            goal.id = border->getId();
            client_book_border_robot.sendGoal(goal);
          }

          if (ImGui::Button("Release (Robot)")) {
            integration::ReleaseRobotStaticBorderGoal goal;
            goal.id = border->getId();
            if (checkboxValue) {
              goal.status = -1;
            } else {
              goal.status = 0;
            }
            client_release_border_robot.sendGoal(goal);
          }

          if (ImGui::Button("Book (Human)")) {
            integration::BookOperatorStaticBorderGoal goal;
            goal.id = border->getId();
            client_book_border_human.sendGoal(goal);
          }

          if (ImGui::Button("Release (Human)")) {
            integration::ReleaseOperatorStaticBorderGoal goal;
            goal.id = border->getId();
            if (checkboxValue) {
              goal.status = -1;
            } else {
              goal.status = 0;
            }
            client_release_border_human.sendGoal(goal);
          }

          ImGui::EndTabItem();
        }
      }
    }
    ImGui::EndTabBar();
  }

  ImGui::End();
}

void Projector::show_debug_buttons() {
  ImGui::Begin("Buttons");
  if (ImGui::BeginTabBar("ButtonTabBar")) {
    for (auto &zone : display_areas) {
      std::vector<std::shared_ptr<Button>> buttons;
      zone->fetchButtons(buttons);
      for (auto &button : buttons) {
        if (ImGui::BeginTabItem(button->get_name().c_str())) {
          ImGui::TextUnformatted(
              (std::string("This is the content of ") + button->get_name())
                  .c_str());
          ImGui::EndTabItem();
        }
      }
    }
    ImGui::EndTabBar();
  }

  ImGui::End();
}

void Projector::show_debug_hands() {
  ImGui::Begin("Hand dection");
  ImGui::Text("Hand detection msg counter: %d", hand_detection_counter);
  ImGui::Text("Time interval between messages: %.2f seconds",
              interval_in_seconds_);
  ImGui::End();
}

void Projector::show_moving_table() {
  ImGui::Begin("Table dection");
  ImGui::Text("Table detection msg counter: %d", table_detection_counter);
  ImGui::Text("Time interval between messages: %.2f seconds",
              table_interval_in_seconds_);
  ImGui::End();
}

void Projector::show_debug_object_detection() {
  ImGui::Begin("Object dection");
  static int thresholdValueSlider = 10;
  static int nonZeroThresholdValueSlider = 10;
  static int noiseReductionValueSlider = 3;
  ImGui::SliderInt("Depth object threshold", &thresholdValueSlider, 0, 200);
  ImGui::SliderInt("Non zero detection threshold", &nonZeroThresholdValueSlider,
                   0, 200);
  ImGui::SliderInt("Noise reduction", &noiseReductionValueSlider, 0, 10);

  if (ImGui::Button("Scan workspace")) {
    ROS_INFO("clicked");
    scan = true;

    std_msgs::Int32 threshold_msg;
    threshold_msg.data = thresholdValueSlider;
    threshold_pub.publish(threshold_msg);

    std_msgs::Int32 non_zero_threshold_msg;
    non_zero_threshold_msg.data = nonZeroThresholdValueSlider;
    non_zero_threshold_pub.publish(non_zero_threshold_msg);

    std_msgs::Int32 noise_reduction_msg;
    noise_reduction_msg.data = noiseReductionValueSlider;
    noise_recuction_pub.publish(noise_reduction_msg);
    ros::Duration(0.5).sleep();
  }

  ImGui::End();
}

void Projector::show_projected_image() {
  ImGui::Begin("Projected image");

  GLuint textureID = cvMatToTexture(combined);
  ImGui::Image((ImTextureID)(uintptr_t)textureID, ImVec2(1024, 768));

  ImGui::End();
}

void Projector::show_layer_manager() {
  ImGui::Begin("Layer Manager");
  ImGui::Text("Set visibility of each layer.");

  for (auto it = layers.begin(); it != layers.end(); ++it) {
    const std::string &name = it->first;
    Layer &layer = it->second;

    ImGui::BeginChild(("LayerCheckboxFrame##" + name).c_str(), ImVec2(0, 30),
                      true, ImGuiWindowFlags_NoTitleBar);
    ImGui::Checkbox(name.c_str(), &layer.visible);
    ImGui::EndChild();
  }

  ImGui::End();
}

void Projector::show_element_creator() {
  static char buttonName[128] = "";
  static char borderName[128] = "";
  static char projectionZone[128] = "";
  static float sliderValue = 0.0f;
  static bool checkboxValue = false;

  ImGui::Begin("Element creator");
  ImGui::Text("Add borders or buttons.");
  ImGui::NewLine();
  if (ImGui::Button("Add Border Layout")) {
    ImGui::OpenPopup("CreateBorderLayoutPopup");
  }

  ImGui::NewLine();
  if (ImGui::Button("Add Border")) {
    ImGui::OpenPopup("CreateBorderPopup");
  }

  ImGui::NewLine();

  if (ImGui::Button("Add Button")) {
    ImGui::OpenPopup("CreateButtonPopup");
  }

  if (ImGui::BeginPopup("CreateBorderLayoutPopup")) {
    static int rowValueSlider = 0;
    static int columnValueSlider = 0;

    integration::SetLayoutStaticBordersGoal layout_goal;

    std::string request_id = "09d-d09d9-fgd";
    layout_goal.request_id = request_id;

    layout_goal.safety_factor = 1.0;

    layout_goal.status_booked.r = 1.0;
    layout_goal.status_booked.g = 0.0;
    layout_goal.status_booked.b = 0.0;
    layout_goal.status_booked.a = 0.0;

    layout_goal.status_free.r = 0.0;
    layout_goal.status_free.g = 1.0;
    layout_goal.status_free.b = 0.0;
    layout_goal.status_free.a = 0.0;

    layout_goal.status_operator.r = 0.0;
    layout_goal.status_operator.g = 0.0;
    layout_goal.status_operator.b = 1.0;
    layout_goal.status_operator.a = 0.0;

    ImGui::Text("Set your options:");

    ImGui::SliderInt("Number of rows", &rowValueSlider, 1, 5);
    ImGui::SliderInt("Number of columns", &columnValueSlider, 1, 5);

    ImGui::Checkbox("Book adjacent", &checkboxValue);

    if (ImGui::Button("Create")) {
      row_layout = rowValueSlider;
      column_layout = columnValueSlider;
      layout_goal.size_cols = columnValueSlider;
      layout_goal.size_rows = rowValueSlider;
      layout_goal.book_adjacent = checkboxValue;
      ROS_INFO("Sending goal...");
      ac.sendGoal(layout_goal);

      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }

  if (ImGui::BeginPopup("CreateBorderPopup")) {
    integration::SetSafetyBorderProjectionGoal msg;
    static std::string area;
    static int rowValue = 0;
    static int columnValue = 0;
    ImGui::InputText("Border name", borderName, sizeof(borderName));

    static int selected_index = 0;
    std::vector<const char *> keys;
    for (const auto &pair : layers) {
      keys.push_back(pair.first.c_str());
    }

    if (ImGui::Combo("Select a projection area", &selected_index, keys.data(),
                     keys.size())) {
      area = keys[selected_index];
    }

    ImGui::InputInt("Row", &rowValue, 1, 5);
    ImGui::InputInt("Column", &columnValue, 1, 5);

    msg.border.polygon.points.clear();
    msg.border.header.frame_id = "base";
    msg.border.header.stamp = ros::Time::now();

    msg.border_topic = "";
    msg.border_color.r = 0.0;
    msg.border_color.g = 1.0;
    msg.border_color.b = 0.0;
    msg.border_color.a = 0.0;
    msg.is_filled = false;
    msg.thickness = 1;
    msg.lifetime.fromNSec(0);
    msg.track_violations = true;

    if (ImGui::Button("Create")) {
      msg.request_id = borderName;

      msg.position_row = rowValue;
      msg.position_col = columnValue;
      msg.zone = area;

      ROS_INFO("Sending goal...");
      client_border.sendGoal(msg);
      ImGui::CloseCurrentPopup();
    }

    if (ImGui::Button("Create all borders")) {
      int count = 1;
      for (int i = 1; i <= row_layout; i++) {
        for (int j = 1; j <= column_layout; j++) {
          msg.request_id = borderName + std::to_string(count);
          msg.position_row = i;
          msg.position_col = j;
          msg.zone = area;

          goalQueue.push(msg);

          count++;
        }
      }
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }

  if (ImGui::BeginPopup("CreateButtonPopup")) {
    static std::string button_area;
    ImGui::InputText("Button name", buttonName, sizeof(buttonName));

    integration::SetVirtualButtonsProjectionGoal goal;

    static int selected_index = 0;
    std::vector<const char *> keys;
    for (const auto &pair : layers) {
      keys.push_back(pair.first.c_str());
    }

    if (ImGui::Combo("Select a projection area", &selected_index, keys.data(),
                     keys.size())) {
      button_area = keys[selected_index];
    }

    // Set button color (RGBA)
    goal.virtual_button.button_color.r = 0.0;
    goal.virtual_button.button_color.g = 0.0;
    goal.virtual_button.button_color.b = 1.0;
    goal.virtual_button.button_color.a = 1.0;

    // Set text color (RGBA)
    goal.virtual_button.text_color.r = 1.0;
    goal.virtual_button.text_color.g = 1.0;
    goal.virtual_button.text_color.b = 1.0;
    goal.virtual_button.text_color.a = 1.0;

    // Set button center position
    static float xValue = 0.0f;
    static float yValue = 0.0f;
    static float radius = 0.0f;
    ImGui::InputFloat("x", &xValue, 0.0f, 1.0f, "%.1f");
    ImGui::InputFloat("y", &yValue, 0.0f, 1.0f, "%.1f");
    ImGui::InputFloat("radius", &radius, 0.0f, 100.0f, "%.1f");

    // Set additional button properties

    goal.virtual_button.hidden = false;

    if (ImGui::Button("Create")) {
      goal.request_id = buttonName;
      goal.zone = button_area;
      goal.virtual_button.id = buttonName;
      goal.virtual_button.zone = button_area;
      goal.virtual_button.name = buttonName;
      goal.virtual_button.text = buttonName;
      goal.virtual_button.center.position.x = xValue;
      goal.virtual_button.center.position.y = yValue;
      goal.virtual_button.center.position.z = 0.0;
      goal.virtual_button.radius = radius;

      buttonQueue.push(goal);
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }
  ImGui::End();
}

void Projector::show_node_starter() {
  ImGui::Begin("Start nodes");

  if (ImGui::Button("Sart tf")) {
    tf = true;
  }

  if (ImGui::Button("Start smart interface")) {
    smart_interface_pub.publish(std_msgs::Empty());
  }

  ImGui::End();
}

void Projector::launchTfNode() {
  std::string command = "roslaunch tuni_whitegoods tuni_tf.launch";
  int result = std::system(command.c_str());

  if (result == 0) {
    ROS_INFO("Launch file started successfully.");
  } else {
    ROS_ERROR("Failed to start launch file.");
  }
}

void Projector::update_gui() {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  show_node_starter();
  if (tf) {
    std::thread(&Projector::launchTfNode, this).detach();
    tf = false;
  }
  show_projected_image();
  show_layer_manager();
  show_element_creator();
  show_debug_borders();

  if (!goalQueue.empty()) {
    // Get the next goal from the queue
    integration::SetSafetyBorderProjectionGoal currentGoal = goalQueue.front();

    ROS_INFO("Sending goal: %s", currentGoal.request_id.c_str());
    client_border.sendGoal(currentGoal);
    ros::Duration(0.2).sleep();
    goalQueue.pop();
  }

  show_debug_buttons();
  if (!buttonQueue.empty()) {
    // Get the next goal from the queue
    integration::SetVirtualButtonsProjectionGoal currentGoal =
        buttonQueue.front();

    ROS_INFO("Sending goal: %s", currentGoal.request_id.c_str());
    project_client.sendGoal(currentGoal);
    ros::Duration(0.2).sleep();
    buttonQueue.pop();
  }

  show_debug_hands();
  show_debug_object_detection();
  show_moving_table();
  if (scan) {
    integration::ListStaticBordersStatus srv;

    std::thread([this, srv]() mutable {
      if (client_detection.call(srv)) {
        ROS_INFO("Asynchronous response received");
      } else {
        ROS_ERROR("Failed to call service");
      }
    }).detach();

    scan = false;
  }

  ImGui::Render();
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(window);
  glfwPollEvents();
}

void Projector::updateButtons(
    const std::vector<std::shared_ptr<Button>> &buttons,
    std::shared_ptr<cv::Mat> layer) {
  for (auto &button : buttons) {
    cv::circle(*layer, button->center_projected_point, button->radius,
               button->btn_color, -1);
  }
}

void Projector::updateBorders(
    const std::vector<std::shared_ptr<StaticBorder>> &borders,
    std::shared_ptr<cv::Mat> layer) {
  for (auto &border : borders) {
    cv::rectangle(
        *layer, border->top_left_proj_point, border->bottom_right_proj_point,
        cv::Scalar(border->border_color.b * 255, border->border_color.g * 255,
                   border->border_color.r * 255),
        border->thickness * 2, cv::LINE_8);
  }
}

void Projector::updateHands(const std::vector<std::shared_ptr<Hand>> &hands) {}

void Projector::updateDisplayAreas(
    const std::vector<std::shared_ptr<DisplayArea>> &zones) {
  for (auto &zone : zones) {
    if (!(zone->name == "projector" || zone->name == "camera")) {
      cv::Point tl(zone->projector_frame_area[0].x,
                   zone->projector_frame_area[0].y);
      cv::Point tr(zone->projector_frame_area[1].x,
                   zone->projector_frame_area[1].y);
      cv::Point br(zone->projector_frame_area[2].x,
                   zone->projector_frame_area[2].y);
      cv::Point bl(zone->projector_frame_area[3].x,
                   zone->projector_frame_area[3].y);

      std::vector<cv::Point> rectanglePoints = {tl, tr, br, bl};

      const cv::Scalar color(255, 255, 255);
      const int thickness = 10;
      bool visibility = layers[zone->name].visible;

      layers[zone->name] = {
          std::make_shared<cv::Mat>(cv::Mat::zeros(
              projector_resolution[1], projector_resolution[0], CV_8UC3)),
          visibility};

      cv::polylines(*layers[zone->name].mat, rectanglePoints, true,
                    cv::Scalar(255), 10, cv::LINE_8);

      std::vector<std::shared_ptr<Button>> buttons;
      zone->fetchButtons(buttons);
      updateButtons(buttons, layers[zone->name].mat);

      std::vector<std::shared_ptr<StaticBorder>> borders;
      zone->fetchBorders(borders);
      updateBorders(borders, layers[zone->name].mat);
    }
  }
  combined = layers["background"].mat->clone();
  for (auto it = layers.begin(); it != layers.end(); ++it) {
    if (it->second.visible) {
      cv::bitwise_or(combined, *(it->second.mat), combined);
    }
  }

  update_gui();
  project_image();
}

void Projector::project_image() {
  cv::imshow(OPENCV_WINDOW, combined);
  cv::waitKey(1);
}

void Projector::handDetectionCallback(
    const tuni_whitegoods_msgs::HandsState::ConstPtr &msg) {
  ros::Time current_msg_time = ros::Time::now();
  if (!last_msg_time_.isZero()) {
    ros::Duration interval = current_msg_time - last_msg_time_;
    interval_in_seconds_ = interval.toSec();
  }
  last_msg_time_ = current_msg_time;

  hand_detection_counter++;
}

void Projector::tableDetectionCallback(
    const tuni_whitegoods_msgs::DynamicArea::ConstPtr &msg) {
  ros::Time current_msg_time = ros::Time::now();
  if (!last_table_msg_time_.isZero()) {
    ros::Duration interval = current_msg_time - last_table_msg_time_;
    table_interval_in_seconds_ = interval.toSec();
  }
  last_table_msg_time_ = current_msg_time;

  table_detection_counter++;
}