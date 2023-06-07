// #ifdef DEPTH_IMAGE_H
// #define DEPTH_IMAGE_H

#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
class DepthImage : public rclcpp::Node {

public:
  DepthImage(rs2::pipeline &pipe, rs2::align &align) : Node("Depth_image_publisher") {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_rect_raw", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [&]() {
      // rs2::frameset frames = pipe.wait_for_frames();

      // // Publish the depth image
      // rs2::depth_frame depth_frame = frames.get_depth_frame();
      // sensor_msgs::msg::Image depth_image;
      // depth_image.header.frame_id = "realsense_camera";
      // depth_image.header.stamp = this->now();
      // depth_image.width = depth_frame.get_width();
      // depth_image.height = depth_frame.get_height();
      // depth_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      // depth_image.step = depth_image.width * sizeof(uint16_t);
      // depth_image.data.resize(depth_image.height * depth_image.step);
      // memcpy(depth_image.data.data(), depth_frame.get_data(),
      //        depth_image.data.size());
      // publisher_->publish(depth_image);

      rs2::frameset frames = pipe.wait_for_frames();
      //frames = align.process(frames);
      
      rs2::depth_frame depth_frame = frames.get_depth_frame();
      
      cv_bridge::CvImage cv_depth;
      cv_depth.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      cv_depth.image = cv::Mat(cv::Size(depth_frame.get_width(), depth_frame.get_height()),
                             CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
      auto msg_depth = cv_depth.toImageMsg();
      publisher_->publish(*msg_depth);
    });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
// #endif // DEPTH_IMAGE_H
