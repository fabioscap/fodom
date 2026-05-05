#include "fodom_ros/fodom_ros.h"

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace fodom_ros {

FodomROS::FodomROS(const rclcpp::NodeOptions& options)
    : Node("fodom_ros", options) {
  init_params();
  std::cerr << "DIOPORCO\n";
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
  keyframe_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("~/keyframe", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  path_msg_.header.frame_id = odom_frame_;

  pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", rclcpp::SensorDataQoS(),
      std::bind(&FodomROS::pc_cb, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(),
      std::bind(&FodomROS::imu_cb, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "fodom_ros node initialized");
}

void FodomROS::imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
  const double stamp = rclcpp::Time(msg->header.stamp).seconds();
  fodom::ImuMeasurement imu;
  imu.accel =
      fodom::Vec3(msg->linear_acceleration.x, msg->linear_acceleration.y,
                  msg->linear_acceleration.z);
  imu.gyro = fodom::Vec3(msg->angular_velocity.x, msg->angular_velocity.y,
                         msg->angular_velocity.z);
  odom_.feed_imu(stamp, imu);
}

void FodomROS::pc_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  const auto t0 = std::chrono::high_resolution_clock::now();
  const size_t n = msg->width * msg->height;
  cloud_.clear();
  cloud_.reserve(n);

  const double min_r2 = min_range_ * min_range_;
  const double max_r2 = max_range_ * max_range_;

  sensor_msgs::PointCloud2ConstIterator<float> x_it(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(*msg, "z");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> i_it(*msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> t_it(*msg, "t");

  for (size_t i = 0; i < n; ++i, ++x_it, ++y_it, ++z_it, ++i_it, ++t_it) {
    if (!std::isfinite(*x_it) || !std::isfinite(*y_it) || !std::isfinite(*z_it))
      continue;
    const fodom::Vec3 pt(*x_it, *y_it, *z_it);
    const double r2 = pt.squaredNorm();
    if (r2 < min_r2 || r2 > max_r2) continue;
    if (*i_it < min_intensity_) continue;
    cloud_.push_back({pt, *t_it});
  }

  const auto t1 = std::chrono::high_resolution_clock::now();
  const double stamp = rclcpp::Time(msg->header.stamp).seconds();

  const auto* kf_before = odom_.keyframe.get();
  odom_.feed_cloud(stamp, cloud_);
  const bool new_keyframe = odom_.keyframe.get() != kf_before;

  const auto t2 = std::chrono::high_resolution_clock::now();

  if (!odom_.keyframe) return;

  const fodom::PoseType pose =
      odom_.ekf.pose() * odom_.config.imu_in_lidar.inverse();
  const Eigen::Quaterniond q(pose.rotation());
  const Eigen::Vector3d t = pose.translation();
  const auto ros_stamp = msg->header.stamp;

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = ros_stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = sensor_frame_;
    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }

  if (publish_odom_) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = ros_stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = sensor_frame_;
    odom_msg.pose.pose.position.x = t.x();
    odom_msg.pose.pose.position.y = t.y();
    odom_msg.pose.pose.position.z = t.z();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_pub_->publish(odom_msg);
  }

  if (publish_path_) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros_stamp;
    pose_stamped.header.frame_id = odom_frame_;
    pose_stamped.pose.position.x = t.x();
    pose_stamped.pose.position.y = t.y();
    pose_stamped.pose.position.z = t.z();
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();
    path_msg_.header.stamp = ros_stamp;
    path_msg_.poses.push_back(pose_stamped);
    path_pub_->publish(path_msg_);
  }

  if (publish_keyframe_ && new_keyframe && odom_.keyframe) {
    std::vector<fodom::Vec3> leaves;
    fodom::get_leaves(*odom_.keyframe, leaves);

    sensor_msgs::msg::PointCloud2 kf_msg;
    kf_msg.header.stamp = rclcpp::Time(0);
    kf_msg.header.frame_id = odom_frame_;
    kf_msg.height = 1;
    kf_msg.width = static_cast<uint32_t>(leaves.size());
    kf_msg.is_dense = true;
    kf_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(kf_msg);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(leaves.size());

    sensor_msgs::PointCloud2Iterator<float> x_out(kf_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> y_out(kf_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> z_out(kf_msg, "z");
    for (const auto& p : leaves) {
      const fodom::Vec3 pw =
          odom_.keyframe_pose * odom_.config.imu_in_lidar.inverse() * p;
      *x_out = static_cast<float>(pw.x());
      *y_out = static_cast<float>(pw.y());
      *z_out = static_cast<float>(pw.z());
      ++x_out;
      ++y_out;
      ++z_out;
    }
    keyframe_pub_->publish(kf_msg);
  }

  const auto dt_filter =
      std::chrono::duration<double, std::milli>(t1 - t0).count();
  const auto dt_icp =
      std::chrono::duration<double, std::milli>(t2 - t1).count();
  RCLCPP_INFO(get_logger(), "filter: %.2f ms | icp: %.2f ms | %zu pts",
              dt_filter, dt_icp, cloud_.size());
}

void FodomROS::init_params() {
  min_range_ = declare_parameter("min_range", 0.3);
  max_range_ = declare_parameter("max_range", 100.0);
  min_intensity_ = static_cast<size_t>(declare_parameter("min_intensity", 0));

  odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
  sensor_frame_ = declare_parameter("sensor_frame", std::string(""));

  publish_tf_ = declare_parameter("publish_tf", true);
  publish_odom_ = declare_parameter("publish_odom", true);
  publish_path_ = declare_parameter("publish_path", true);
  publish_keyframe_ = declare_parameter("publish_keyframe", false);

  odom_.config.kdtree.max_points_per_leaf =
      static_cast<size_t>(declare_parameter("max_points_per_leaf", 20));
  odom_.config.kdtree.max_depth =
      static_cast<size_t>(declare_parameter("max_depth", 16));
  odom_.config.kdtree.max_parallel_level =
      static_cast<size_t>(declare_parameter("max_parallel_level", 3));

  odom_.config.icp.n_iters =
      static_cast<size_t>(declare_parameter("icp_n_iters", 10));
  odom_.config.icp.damping = declare_parameter("icp_damping", 0.0);
  odom_.config.icp.max_correspondence_dist =
      declare_parameter("icp_max_correspondence_dist", 1.0);

  odom_.config.kf_ratio = declare_parameter("keyframe_min_ratio", 0.3);

  auto& ekf = odom_.ekf.config;
  ekf.accel_noise = declare_parameter("ekf.accel_noise", 0.1);
  ekf.gyro_noise = declare_parameter("ekf.gyro_noise", 0.01);
  ekf.accel_bias_rw = declare_parameter("ekf.accel_bias_rw", 1e-4);
  ekf.gyro_bias_rw = declare_parameter("ekf.gyro_bias_rw", 1e-5);
  ekf.pos_noise = declare_parameter("ekf.pos_noise", 0.05);
  ekf.rot_noise = declare_parameter("ekf.rot_noise", 0.01);
  ekf.calib_samples =
      static_cast<size_t>(declare_parameter("ekf.calib_samples", 200));
  ekf.still_window = declare_parameter("ekf.still_window", 0.5);
  ekf.still_accel_var = declare_parameter("ekf.still_accel_var", 0.01);
  ekf.still_gyro_var = declare_parameter("ekf.still_gyro_var", 1e-4);
  ekf.still_accel_thr = declare_parameter("ekf.still_accel_thr", 0.5);
  ekf.still_gyro_thr = declare_parameter("ekf.still_gyro_thr", 0.05);
  ekf.do_zupt = declare_parameter("ekf.do_zupt", false);
  ekf.zupt_noise = declare_parameter("ekf.zupt_noise", 1e-4);
  ekf.init_pos_var = declare_parameter("ekf.init_pos_var", 1e-3);
  ekf.init_vel_var = declare_parameter("ekf.init_vel_var", 1e-3);
  ekf.init_rot_var = declare_parameter("ekf.init_rot_var", 1e-3);
  ekf.init_accel_bias_var = declare_parameter("ekf.init_accel_bias_var", 1e-3);
  ekf.init_gyro_bias_var = declare_parameter("ekf.init_gyro_bias_var", 1e-6);
  const auto g = declare_parameter("ekf.gravity", std::vector<double>{0.0, 0.0, -9.81});
  ekf.gravity = fodom::Vec3(g[0], g[1], g[2]);

  // row-major 4x4 homogeneous transform
  const auto m = declare_parameter(
      "imu_in_lidar",
      std::vector<double>{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
  Eigen::Matrix4d mat;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) mat(r, c) = m[r * 4 + c];
  odom_.config.imu_in_lidar.matrix() = mat;
}

}  // namespace fodom_ros

RCLCPP_COMPONENTS_REGISTER_NODE(fodom_ros::FodomROS)
