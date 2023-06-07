#include <chrono>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class RealSenseImageCaptureNode : public rclcpp::Node
{
public:
  RealSenseImageCaptureNode() : Node("realsense_image_capture")
  {
    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", 10, std::bind(&RealSenseImageCaptureNode::color_callback, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_rect_raw", 10, std::bind(&RealSenseImageCaptureNode::depth_callback, this, std::placeholders::_1));

    // Create the output directory if it doesn't exist
    output_dir_ = "/home/khadas/picture/";
    if (!std::filesystem::exists(output_dir_)) {
      std::filesystem::create_directory(output_dir_);
    }

    timer_ = create_wall_timer(std::chrono::seconds(3), std::bind(&RealSenseImageCaptureNode::timer_callback, this));
  }

private:
  void color_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert the color image to OpenCV format
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat color_image = cv_ptr->image;

    // Save the color image as a file
    std::string filename = output_dir_ + "/color_" + std::to_string(std::time(nullptr)) + ".jpg";
    cv::imwrite(filename, color_image);
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert the depth image to OpenCV format
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "passthrough");
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat depth_image = cv_ptr->image;

    // Normalize the depth values for visualization
    cv::normalize(depth_image, depth_image, 0, 255, cv::NORM_MINMAX, CV_8U);

    // Save the depth image as a file
    std::string filename = output_dir_ + "/depth_" + std::to_string(std::time(nullptr)) + ".jpg";
    cv::imwrite(filename, depth_image);
  }

  void timer_callback()
  {
    RCLCPP_INFO(get_logger(), "Capturing images...");

    // Capture a color and depth image
    rclcpp::spin_some();
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string output_dir_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc,
