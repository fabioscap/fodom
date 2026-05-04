#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "fodom/odom.h"

namespace fodom_ros {

class FodomROS : public rclcpp::Node {
 public:
  explicit FodomROS(const rclcpp::NodeOptions& options);

 private:
  void pc_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg);
  void init_params();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double min_range_;
  double max_range_;
  size_t min_intensity_;

  std::string odom_frame_;
  std::string sensor_frame_;

  bool publish_tf_;
  bool publish_odom_;
  bool publish_path_;
  bool publish_keyframe_;

  nav_msgs::msg::Path path_msg_;

  fodom::Odom      odom_;
  fodom::PCloudType cloud_;
};

}  // namespace fodom_ros
