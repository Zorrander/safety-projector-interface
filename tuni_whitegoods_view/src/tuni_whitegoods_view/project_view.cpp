#include "tuni_whitegoods_view/project_view.h"

#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Quaternion.h>

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
      nh_(nh) {
  if (!ros::param::get("shiftX", shift)) {
    shift = 0;  // Default value
    ROS_WARN("Parameter 'shiftX' not found, using default value 0.");
  }
  ros::param::get("projector_resolution", projector_resolution);

  client_detection = nh->serviceClient<integration::ListStaticBordersStatus>(
      "/execution/projector_interface/integration/services/"
      "list_static_border_status");

  if (!client_detection.waitForExistence(ros::Duration(5.0))) {
    ROS_ERROR("Service not available after waiting");
  } else {
    ROS_INFO("Service available");
  }
  layers["background"] = {
      std::make_shared<cv::Mat>(cv::Mat::zeros(
          projector_resolution[1], projector_resolution[0], CV_8UC3)),
      true};

  cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
  cv::moveWindow(OPENCV_WINDOW, shift, 0);
  cv::setWindowProperty(OPENCV_WINDOW, cv::WND_PROP_FULLSCREEN,
                        cv::WINDOW_FULLSCREEN);

  // Initialize OpenGL, GLFW, and ImGui
  initializeGLFWandOpenGL();

  // Create a GLFW window for rendering
  window = glfwCreateWindow(2300, 1300, "Layer Manager", nullptr, nullptr);
  glfwMakeContextCurrent(window);
  glewInit();

  // Initialize ImGui
  initializeImGui(window);

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
    // update_gui();
    // project_image();
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
    ImGui::EndTabBar();  // End the tab bar
  }

  ImGui::End();  // End the window
}

void Projector::show_debug_hands() {}

void Projector::show_debug_object_detection() {
  ImGui::Begin("Object dection");
  if (ImGui::Button("Scan workspace")) {
    ROS_INFO("clicked");
    if (client_detection.call(srv)) {
      ROS_INFO("Service call successful. Border statuses:");
      for (const auto &border_status : srv.response.status_borders) {
        ROS_INFO("Border ID: %s, Status: %d", border_status.id.c_str(),
                 border_status.status);
      }
    } else {
      ROS_ERROR("Failed to call service getBordersService");
    }
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

  // Iterate through layers and create checkboxes
  for (auto it = layers.begin(); it != layers.end(); ++it) {
    const std::string &name = it->first;
    Layer &layer = it->second;

    // Frame for the checkbox
    ImGui::BeginChild(("LayerCheckboxFrame##" + name).c_str(), ImVec2(0, 30),
                      true, ImGuiWindowFlags_NoTitleBar);
    ImGui::Checkbox(name.c_str(), &layer.visible);
    ImGui::EndChild();
  }

  ImGui::End();
}

void Projector::show_element_creator() {
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
    static int rowValueSlider = 0;     // Slider integer value
    static int columnValueSlider = 0;  // Slider integer value

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

    // Button to validate choices
    if (ImGui::Button("Create")) {
      layout_goal.size_cols = columnValueSlider;
      layout_goal.size_rows = rowValueSlider;
      layout_goal.book_adjacent = checkboxValue;
      ROS_INFO("Sending goal...");
      ac.sendGoal(layout_goal);

      // Close the popup after validation (optional)
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

    ImGui::EndPopup();
  }

  if (ImGui::BeginPopup("CreateButtonPopup")) {
    ImGui::Text("Set your options:");

    // Slider for float value
    ImGui::SliderFloat("Slider Value", &sliderValue, 0.0f, 100.0f);

    // Checkbox for a boolean value
    ImGui::Checkbox("Checkbox", &checkboxValue);

    // Button to validate choices
    if (ImGui::Button("Validate")) {
      // Process the input data
      ImGui::Text("Slider Value: %.1f", sliderValue);
      ImGui::Text("Checkbox is %s", checkboxValue ? "checked" : "unchecked");
      // You can add additional processing logic here

      // Close the popup after validation (optional)
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }
  ImGui::End();
}

void Projector::update_gui() {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  show_projected_image();
  show_layer_manager();
  show_element_creator();
  show_debug_borders();
  show_debug_buttons();
  show_debug_hands();
  show_debug_object_detection();
  // Render ImGui
  ImGui::Render();
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  // Swap OpenGL buffers
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
