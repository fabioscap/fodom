#include "fodom_ros/fodom_ros.h"

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace fodom_ros {

FodomROS::FodomROS(const rclcpp::NodeOptions& options)
    : Node("fodom_ros", options) {
  init_params();

  // Publishers
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
  keyframe_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("~/keyframe", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  path_msg_.header.frame_id = odom_frame_;

  // Subscriber
  pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", rclcpp::SensorDataQoS(),
      std::bind(&FodomROS::pc_cb, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "odom_ros node initialized");
}

void FodomROS::pc_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {}

void FodomROS::init_params() {
  min_range_ = declare_parameter("min_range", 0.3);
  max_range_ = declare_parameter("max_range", 100.0);
  min_intensity_ = declare_parameter("min_intensity", 0);

  // Frame IDs
  odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
  sensor_frame_ = declare_parameter("sensor_frame", std::string(""));

  // Publishing toggles
  publish_tf_ = declare_parameter("publish_tf", true);
  publish_odom_ = declare_parameter("publish_odom", true);
  publish_path_ = declare_parameter("publish_path", true);
  publish_keyframe_ = declare_parameter("publish_keyframe", false);
}

}  // namespace fodom_ros

RCLCPP_COMPONENTS_REGISTER_NODE(fodom_ros::FodomROS)
