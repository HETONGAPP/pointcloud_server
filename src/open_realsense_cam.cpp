#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <memory>
#include <opencv2/highgui.hpp>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <algorithm>
#include <tuple>
using PointT = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;
using cloud_pointer = PointCloudT::Ptr;
using prevCloud = PointCloudT::Ptr;

std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture,
                                      rs2::texture_coordinate Texture_XY) {
  // Get Width and Height coordinates of texture
  int width = texture.get_width();   // Frame width in pixels
  int height = texture.get_height(); // Frame height in pixels

  // Normals to Texture Coordinates conversion
  int x_value = std::min(std::max(int(Texture_XY.u * width + .5f), 0), width - 1);
  int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

  int bytes =
      x_value * texture.get_bytes_per_pixel(); // Get # of bytes per pixel
  int strides =
      y_value * texture.get_stride_in_bytes(); // Get line width in bytes
  int Text_Index = (bytes + strides);

  const auto New_Texture =
      reinterpret_cast<const uint8_t *>(texture.get_data());

  // RGB components to save in tuple
  int NT1 = New_Texture[Text_Index];
  int NT2 = New_Texture[Text_Index + 1];
  int NT3 = New_Texture[Text_Index + 2];

  return std::tuple<int, int, int>(NT1, NT2, NT3);
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud object with depth and RGB data
// from a single frame captured using the Realsense.
//===================================================
cloud_pointer PCL_Conversion(const rs2::points &points,
                             const rs2::video_frame &color) {

  // Object Declaration (Point Cloud)
  cloud_pointer cloud(new PointCloudT);

  // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
  std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

  //================================
  // PCL Cloud Object Configuration
  //================================
  // Convert data captured from Realsense camera to Point Cloud
  auto sp = points.get_profile().as<rs2::video_stream_profile>();

  cloud->width = static_cast<uint32_t>(sp.width());
  cloud->height = static_cast<uint32_t>(sp.height());
  cloud->is_dense = false;
  cloud->points.resize(points.size());

  auto Texture_Coord = points.get_texture_coordinates();
  auto Vertex = points.get_vertices();

  // Iterating through all points and setting XYZ coordinates
  // and RGB values
  for (int i = 0; i < points.size(); i++) {
    //===================================
    // Mapping Depth Coordinates
    // - Depth data stored as XYZ values
    //===================================
    cloud->points[i].x = Vertex[i].x;
    cloud->points[i].y = Vertex[i].y;
    cloud->points[i].z = Vertex[i].z;

    // Obtain color texture for specific point
    RGB_Color = RGB_Texture(color, Texture_Coord[i]);

    // Mapping Color (BGR due to Camera Model)
    cloud->points[i].r = std::get<2>(RGB_Color); // Reference tuple<2>
    cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
    cloud->points[i].b = std::get<0>(RGB_Color); // Reference tuple<0>
  }

  return cloud; // PCL RGB Point Cloud generated
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("realsense_camera");

  // Declare the RealSense pipeline and configure it for depth and color streams
  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 15);

  // Start the RealSense pipeline
  pipe.start(cfg);

  // Create a publisher for the depth and color images
  auto depth_publisher =
      node->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_rect_raw", 10);
  auto color_publisher =
      node->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", 10);
  auto point_cloud_publisher =
      node->create_publisher<sensor_msgs::msg::PointCloud2>("local_grid_obstacle", 10);

  // Set up a timer to read frames from the RealSense pipeline and publish them
  auto timer = node->create_wall_timer(std::chrono::milliseconds(3), [&]() {
    rs2::frameset frames = pipe.wait_for_frames();

    // Publish the depth image
    rs2::depth_frame depth_frame = frames.get_depth_frame();
    sensor_msgs::msg::Image depth_image;
    depth_image.header.frame_id = "realsense_camera";
    depth_image.header.stamp = node->now();
    depth_image.width = depth_frame.get_width();
    depth_image.height = depth_frame.get_height();
    depth_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    depth_image.step = depth_image.width * sizeof(uint16_t);

    cv::Mat depth_(cv::Size(640, 480), CV_16UC1, (void *)depth_frame.get_data(),
                   cv::Mat::AUTO_STEP);
    depth_.convertTo(depth_, CV_8UC1,
                     255.0 / 1000.0); // Convert to 8-bit depth image
    cv::applyColorMap(depth_, depth_,
                      cv::COLORMAP_JET); // Apply colormap
    sensor_msgs::msg::Image::SharedPtr msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", depth_)
            .toImageMsg();
    depth_image.data.resize(depth_image.height * depth_image.step);
    memcpy(depth_image.data.data(), depth_frame.get_data(),
           depth_image.data.size());
    depth_publisher->publish(*msg);

    // Publish the color image
    rs2::video_frame color_frame = frames.get_color_frame();
    sensor_msgs::msg::Image color_image;
    color_image.header.frame_id = "realsense_camera";
    color_image.header.stamp = node->now();
    color_image.width = color_frame.get_width();
    color_image.height = color_frame.get_height();
    color_image.encoding = sensor_msgs::image_encodings::RGB8;
    color_image.step = color_image.width * 3;
    color_image.data.resize(color_image.height * color_image.step);

    memcpy(color_image.data.data(), color_frame.get_data(),
           color_image.data.size());
    color_publisher->publish(color_image);

    // Convert the depth and color images to a point cloud
    rs2::pointcloud pc;
    pc.map_to(color_frame);
    rs2::points points = pc.calculate(depth_frame);

    PointCloudT::Ptr cloud(new PointCloudT);

    cloud = PCL_Conversion(points, color_frame);

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);

    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    pass.filter(*cloud_filtered);
    cloud->swap(*cloud_filtered);

    // Create a new point cloud to store the downsampled data
    PointCloudT::Ptr cloud_downsampled(new PointCloudT);

    // Create a voxel grid filter object
    // pcl::VoxelGrid<PointT> sor;
    pcl::RandomSample<PointT> rs;

    std::cout << "PCL original points size: " << cloud->size() << std::endl;
    if (cloud->size() >= 2500) {
      // Set the input cloud for the filter
      // sor.setInputCloud(cloud);

      // // Set the voxel grid size
      // sor.setLeafSize(0.03f, 0.03f, 0.03f);

      // sor.filter(*cloud_downsampled);

      // Set the input cloud for the filter
      rs.setInputCloud(cloud);

      // Set the ratio of points to be randomly selected
      rs.setSample(2500);

      // Apply the filter to obtain the downsampled point cloud
      rs.filter(*cloud_downsampled);

      cloud->swap(*cloud_downsampled);
    }

    // Remove NaN values from the point cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    sensor_msgs::msg::PointCloud2 point_cloud;
    pcl::toROSMsg(*cloud, point_cloud);

    point_cloud.header.frame_id = "realsense_camera";
    point_cloud.header.stamp = node->now();
    point_cloud_publisher->publish(point_cloud);
    std::cout << "PCL points size: " << cloud->size() << std::endl;

    // // 检查话题是否有订阅者
    // size_t num_subscribers = node->count_subscribers("point_cloud");

    // 输出订阅者数量
    // std::cout << "Number of subscribers: " << num_subscribers << std::endl;
  });

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
