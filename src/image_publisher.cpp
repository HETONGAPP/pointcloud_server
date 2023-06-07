#include <chrono>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"



class RealsenseImageSubscriber : public rclcpp::Node
{
public:
  RealsenseImageSubscriber() : Node("realsense_image_subscriber")
  {
    color_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", 10, std::bind(&RealsenseImageSubscriber::color_callback, this, std::placeholders::_1));
    depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_rect_raw", 10, std::bind(&RealsenseImageSubscriber::depth_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::seconds(3), std::bind(&RealsenseImageSubscriber::save_images, this));
  }

private:
  void color_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert the color image message to OpenCV format
    cv_bridge::CvImagePtr cv_color;
    cv_color = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat color_image = cv_color->image;

    // Store the color image in the member variable
    color_image_ = color_image.clone();
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert the depth image message to OpenCV format
    cv_bridge::CvImagePtr cv_depth;
    cv_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    // Apply a color map to the depth image
    cv::Mat depth_image = cv_depth->image;

    // Apply a color map to the depth image
    cv::Mat depth_color;
    cv::applyColorMap(depth_image, depth_color, cv::COLORMAP_JET);

    // Store the depth image in the member variable
    depth_image_ = depth_color.clone();
  }

  void save_images()
  {
    // Get the current time as a string
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d-%H-%M-%S");
    std::string time_string = ss.str();
    
    std::string folder_path = "/home/khadas/picture/";

    // Save the color and depth images to files
    std::string color_filename = folder_path+"color_" + time_string + ".png";
    std::string depth_filename = folder_path+"depth_" + time_string + ".png";
    cv::imwrite(color_filename, color_image_);
    cv::imwrite(depth_filename, depth_image_);

    // Print a message to the console
    RCLCPP_INFO(this->get_logger(), "Images saved: %s, %s", color_filename.c_str(), depth_filename.c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Mat color_image_;
  cv::Mat depth_image_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealsenseImageSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
