// #ifdef RGB_IMAGE_H
// #define RGB_IMAGE_H

#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
class RgbImage : public rclcpp::Node {

public:
  RgbImage(rs2::pipeline &pipe, rs2::align &align) : Node("RGB_image_publisher") {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("color", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [&]() {
      // rs2::frameset frames = pipe.wait_for_frames();

      // // Publish the color image
      // rs2::video_frame color_frame = frames.get_color_frame();
      // sensor_msgs::msg::Image color_image;
      // color_image.header.frame_id = "realsense_camera";
      // color_image.header.stamp = this->now();
      // color_image.width = color_frame.get_width();
      // color_image.height = color_frame.get_height();
      // color_image.encoding = sensor_msgs::image_encodings::RGB8;
      // color_image.step = color_image.width * 3;
      // color_image.data.resize(color_image.height * color_image.step);

      // memcpy(color_image.data.data(), color_frame.get_data(),
      //        color_image.data.size());
      // publisher_->publish(color_image);

      // Publish the color image
      
      rs2::frameset frames = pipe.wait_for_frames();
      //auto processed = align.process(frames);
    rs2::video_frame color_frame = frames.get_color_frame();
    sensor_msgs::msg::Image color_image;
    color_image.header.frame_id = "realsense_camera";
    color_image.header.stamp = this->now();
    color_image.width = color_frame.get_width();
    color_image.height = color_frame.get_height();
    color_image.encoding = sensor_msgs::image_encodings::RGB8;
    color_image.step = color_image.width * 3;
    color_image.data.resize(color_image.height * color_image.step);

    memcpy(color_image.data.data(), color_frame.get_data(),
           color_image.data.size());
    publisher_->publish(color_image);
      // Display color image in OpenCV window
      // cv::imshow("RealSense Camera", image);
      //cv::waitKey(1);
    });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
// #endif // RGB_IMAGE_H
