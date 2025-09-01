#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <task_manager_msg/msg/task_status.hpp>
#include <string>

using namespace std::chrono_literals;

class OpenCVPublisher : public rclcpp::Node {
public:
  OpenCVPublisher() : Node("opencv_publisher") {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    timer_ = this->create_wall_timer(200ms, std::bind(&OpenCVPublisher::timer_callback, this));
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "border_color", 10,
      std::bind(&OpenCVPublisher::border_color_callback, this, std::placeholders::_1));

    // Default color is red
    border_color_ = cv::Scalar(0, 0, 255);

    // Create OpenCV window
    cv::namedWindow("Publisher View");
    cv::namedWindow("Border control");
    cv::namedWindow("Mir status control");
    cv::namedWindow("Doosan status control");

    // Create trackbars to control the position of the circle
    cv::createTrackbar("Center X", "Mir status control", &mir_status_top_left_.x, 1920, on_trackbar, this);
    cv::createTrackbar("Center Y", "Mir status control", &mir_status_top_left_.y, 1080, on_trackbar, this);
    cv::createTrackbar("Width", "Mir status control", &mir_rect_width, 500, on_trackbar, this);
    cv::createTrackbar("Height", "Mir status control", &mir_rect_height, 500, on_trackbar, this);
    cv::createTrackbar("Rect Angle", "Mir status control", &mir_status_angle_, 360, on_trackbar, this);


    cv::createTrackbar("Center X", "Doosan status control", &doosan_status_top_left_.x, 1920, on_trackbar, this);
    cv::createTrackbar("Center Y", "Doosan status control", &doosan_status_top_left_.y, 1080, on_trackbar, this);
    cv::createTrackbar("Width", "Doosan status control", &doosan_rect_width, 500, on_trackbar, this);
    cv::createTrackbar("Height", "Doosan status control", &doosan_rect_height, 500, on_trackbar, this);
    cv::createTrackbar("Rect Angle", "Doosan status control", &doosan_status_angle_, 360, on_trackbar, this);

    cv::createTrackbar("Rect Top-Left X", "Border control", &border_top_left_.x, 1920, on_trackbar, this);
    cv::createTrackbar("Rect Top-Left Y", "Border control", &border_top_left_.y, 1080, on_trackbar, this);
    cv::createTrackbar("Rect Bottom-Right X", "Border control", &border_bottom_right_.x, 1920, on_trackbar, this);
    cv::createTrackbar("Rect Bottom-Right Y", "Border control", &border_bottom_right_.y, 1080, on_trackbar, this);
    cv::createTrackbar("Rect Angle", "Border control", &border_angle_, 360, on_trackbar, this);

    cv::createTrackbar("Text size", "Mir status control", &font_scale, 5.0, on_trackbar, this);

    task_status_sub_ = this->create_subscription<task_manager_msg::msg::TaskStatus>(
      "/task/status", 10, std::bind(&OpenCVPublisher::task_status_callback, this, std::placeholders::_1));
    
  }

  ~OpenCVPublisher() {
    cv::destroyWindow("Publisher View");
    cv::destroyWindow("Controls");
  }

private:
  void timer_callback() {
    // Create a blank image (black background)
    cv::Mat image = cv::Mat::zeros(cv::Size(1920, 1080), CV_8UC3);

    // Border
    cv::RotatedRect rotated_rect(cv::Point2f((border_top_left_.x + border_bottom_right_.x) / 2,
                                            (border_top_left_.y + border_bottom_right_.y) / 2),
                                 cv::Size(border_bottom_right_.x - border_top_left_.x, 
                                          border_bottom_right_.y - border_top_left_.y), 
                                 border_angle_);

    cv::Point2f rect_points[4];
    rotated_rect.points(rect_points);
    for (int i = 0; i < 4; i++) {
      cv::line(image, rect_points[i], rect_points[(i+1)%4], border_color_, 3); 
    }

    std::vector<cv::Point> points;
    for (int i = 0; i < 4; ++i) {
      points.push_back(cv::Point(rect_points[i].x, rect_points[i].y));
    }

    cv::fillConvexPoly(image, points.data(), points.size(), border_color_, cv::LINE_AA, 0);

    // Mir status
    draw_text_in_rectangle(image, mir_status_top_left_, cv::Size(mir_rect_width, mir_rect_height), mir_status_angle_, mir_status);
    
    // Doosan status
    draw_text_in_rectangle(image, doosan_status_top_left_, cv::Size(doosan_rect_width, doosan_rect_height), doosan_status_angle_, doosan_status);


    // ==== Show locally ====
    cv::imshow("Publisher View", image);
    cv::waitKey(1);  // Needed for OpenCV window updates

    // ==== Convert to ROS2 Image message ====

    /*
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "camera";

    auto msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    publisher_->publish(*msg);

    RCLCPP_INFO(this->get_logger(), "Published image with border + button");
    */
  }

  static void on_trackbar(int, void* userdata) {
    OpenCVPublisher* publisher = reinterpret_cast<OpenCVPublisher*>(userdata);
    // Trigger a callback to update the image when the trackbar is moved
    publisher->timer_callback();
  }

  void task_status_callback(const task_manager_msg::msg::TaskStatus::SharedPtr msg) {
    mir_status = msg->status[0];
    doosan_status = msg->status[1];
    if (msg->status[0] == "workstation2") {
      border_color_ == cv::Scalar(0, 0, 255);
    } else {
      border_color_ == cv::Scalar(0, 255, 0);
    }
  }

void draw_text_in_rectangle(cv::Mat& image, const cv::Point& top_left, const cv::Size& size, int angle, const std::string& text) {
    cv::Scalar rectangle_color(0, 255, 0);
    // Draw a rectangle (use a filled rectangle to make it look like a button)
    if (text == "working") {
        rectangle_color = cv::Scalar(255, 0, 0); // Blue
    }
    //cv::rectangle(image, top_left, top_left + cv::Point(size.width, size.height), rectangle_color, -1);  // Filled

    cv::Point center(top_left.x + size.width / 2, top_left.y + size.height / 2);

    cv::RotatedRect rotated_rect(center, size, angle);

    cv::Point2f rect_points[4];
    rotated_rect.points(rect_points);
    for (int i = 0; i < 4; i++) {
      cv::line(image, rect_points[i], rect_points[(i+1)%4], rectangle_color, 3); 
    }

    std::vector<cv::Point> points;
    for (int i = 0; i < 4; ++i) {
      points.push_back(cv::Point(rect_points[i].x, rect_points[i].y));
    }

    cv::fillConvexPoly(image, points.data(), points.size(), rectangle_color, cv::LINE_AA, 0);

    // Calculate text size
    int thickness = 2;
    cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, nullptr);

    // Calculate position to center the text inside the rectangle
    cv::Point text_origin(top_left.x + (size.width - text_size.width) / 2, top_left.y + (size.height + text_size.height) / 2);

    // Draw the text inside the rectangle
    cv::putText(image, text, text_origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 255, 255), thickness);
}

void draw_rotated_text_in_rectangle(cv::Mat& image, const cv::Point& top_left, const cv::Size& size, int angle, const std::string& text) {
    // Create a temporary image to hold the rotated text and rectangle
    cv::Mat rotated_image = image.clone();

    // Draw the rectangle and text inside it (before rotation)
    //draw_text_in_rectangle(rotated_image, top_left, size, text);

    // Get the center of the rectangle to rotate around it
    cv::Point center(top_left.x + size.width / 2, top_left.y + size.height / 2);

    // Get the rotation matrix for the text and rectangle
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, angle, 1.0);

    // Rotate only the rectangle and text portion
    cv::Mat rotated_roi = rotated_image(cv::Rect(top_left, size));  // Extract the ROI
    cv::warpAffine(rotated_roi, rotated_roi, rotation_matrix, rotated_roi.size());

    // Now copy the rotated portion back to the image (no need for `copyTo`)
    rotated_image.copyTo(image);

    // Display the rotated image
    image = rotated_image;
}

void border_color_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string color = msg->data;
    if (color == "red") {
      border_color_ = cv::Scalar(0, 0, 255);
    } else if (color == "green") {
      border_color_ = cv::Scalar(0, 255, 0);
    } else if (color == "blue") {
      border_color_ = cv::Scalar(255, 0, 0);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown color: '%s'", color.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Border color changed to %s", color.c_str());
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  rclcpp::Subscription<task_manager_msg::msg::TaskStatus>::SharedPtr task_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  cv::Scalar border_color_; 

  cv::Point border_top_left_; 
  cv::Point border_bottom_right_;
  int border_angle_;  
  
  cv::Point mir_status_top_left_; 
  int mir_rect_width = 100; 
  int mir_rect_height = 100;
  int mir_status_angle_;  
  std::string mir_status = "Idle";

  cv::Point doosan_status_top_left_; 
  int doosan_rect_width = 100; 
  int doosan_rect_height = 100;
  int doosan_status_angle_;
  std::string doosan_status = "Idle";


  int font_scale;

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenCVPublisher>());
  rclcpp::shutdown();
  return 0;
}
