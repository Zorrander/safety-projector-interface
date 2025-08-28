#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

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
  }


  ~OpenCVPublisher() {
    cv::destroyWindow("Publisher View");
  }

private:
  void timer_callback() {
    // Create a blank image (black background)
    cv::Mat image = cv::Mat::zeros(cv::Size(1920, 1080), CV_8UC3);

    // ==== Draw a border ====
    int thickness = 5;
    cv::rectangle(image, cv::Point(0, 0), cv::Point(image.cols-1, image.rows-1),
                  border_color_, thickness); // red border

    // ==== Draw a "button" (circle with text) ====
    cv::Point center(320, 240); // middle of image
    int radius = 60;
    cv::circle(image, center, radius, cv::Scalar(0, 255, 0), -1); // filled green circle

    // Put text inside button
    cv::putText(image, "OK", cv::Point(center.x - 25, center.y + 10),
                cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(255, 255, 255), 3);

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
  rclcpp::TimerBase::SharedPtr timer_;

  cv::Scalar border_color_; 
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenCVPublisher>());
  rclcpp::shutdown();
  return 0;
}
