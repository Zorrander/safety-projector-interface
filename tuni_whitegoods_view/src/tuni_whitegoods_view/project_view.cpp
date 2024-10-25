#include "tuni_whitegoods_view/project_view.h"

#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Quaternion.h>

#include "tf2/utils.h"

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

Projector::Projector(ros::NodeHandle *nh) {
  if (!ros::param::get("shiftX", shift)) {
    shift = 0;  // Default value
    ROS_WARN("Parameter 'shiftX' not found, using default value 0.");
  }
  ros::param::get("projector_resolution", projector_resolution);

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
  window = glfwCreateWindow(1280, 720, "Layer Manager", nullptr, nullptr);
  glfwMakeContextCurrent(window);
  glewInit();

  // Initialize ImGui
  initializeImGui(window);

  ROS_INFO("ProjectorView running");
}

Projector::~Projector() { cv::destroyWindow(OPENCV_WINDOW); }

void Projector::init(std::vector<std::shared_ptr<DisplayArea>> zones) {
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

void Projector::update_gui() {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  ImGui::Begin("Layer Manager and Combined Image");

  ImGui::Columns(2, "SideBySideColumns", false);

  GLuint textureID = cvMatToTexture(combined);

  ImGui::BeginGroup();
  ImGui::Text("Combined Image");
  ImGui::Image((ImTextureID)(uintptr_t)textureID, ImVec2(1024, 768));
  ImGui::EndGroup();

  ImGui::NextColumn();

  ImGui::BeginGroup();
  ImGui::Text("Layer Manager");

  for (auto it = layers.begin(); it != layers.end(); ++it) {
    const std::string &name = it->first;
    Layer &layer = it->second;

    ImGui::Checkbox(name.c_str(), &layer.visible);
    ImGui::SameLine();
  }

  ImGui::EndGroup();

  ImGui::Columns(1);
  ImGui::End();
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
    /*
    cv::rectangle(
        *layer, border->top_left_proj_point, border->bottom_right_proj_point,
        cv::Scalar(border->border_color.b * 255, border->border_color.g * 255,
                   border->border_color.r * 255),
        border->thickness * 2, cv::LINE_8);
    */
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
