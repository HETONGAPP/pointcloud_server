#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/random_sample.h"
#include "pcl/filters/passthrough.h"

class PointCloudSubscriber : public rclcpp::Node {
public:
    PointCloudSubscriber() : Node("pointcloud_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", rclcpp::SensorDataQoS(),
            std::bind(&PointCloudSubscriber::PointCloudCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_pointcloud_topic", 5);
    }

private:
    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert sensor_msgs/PointCloud2 to pcl::PointCloud<PointT>
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *input_cloud);

        // Randomly sample the point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr random_sampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::RandomSample<pcl::PointXYZRGB> rs;
        rs.setSample(8000);
        rs.filter(*random_sampled_cloud);

        // Filter out points outside of the z-range [0.0, 1.0]
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(random_sampled_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.0);
        pass.filter(*output_cloud);

        // Convert pcl::PointCloud<pcl::PointXYZRGB> to sensor_msgs/PointCloud2
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*output_cloud, output_msg);
        output_msg.header = msg->header;

        // Publish the downsampled point cloud
        publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
