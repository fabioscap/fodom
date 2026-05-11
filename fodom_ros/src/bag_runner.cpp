// Offline bag runner: reads a ROS2 bag, runs fodom odometry, writes TUM poses.
//
// Usage:
//   bag_runner <bag_path> <output.tum> [<config.yaml>]
//              [--imu TOPIC] [--points TOPIC]
//              [--min-range M] [--max-range M] [--min-intensity N]
//
// TUM format: timestamp tx ty tz qx qy qz qw  (one line per scan, seconds)

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>

#include "fodom/odom.h"

// ---- config loading --------------------------------------------------------

struct BagConfig {
  fodom::OdomConfig odom;
  double min_range = 1.0;
  double max_range = 30.0;
  size_t min_intensity = 0;
};

static BagConfig load_yaml(const std::string& path) {
  BagConfig bc;
  if (path.empty()) return bc;

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    std::cerr << "Warning: could not parse config '" << path
              << "': " << e.what() << "\n";
    return bc;
  }

  // Accept both fodom_ros.yaml (with node/ros__parameters prefix) and flat
  // YAML.
  YAML::Node p = root;
  if (root["fodom_ros_node"] && root["fodom_ros_node"]["ros__parameters"])
    p = root["fodom_ros_node"]["ros__parameters"];

  auto get_d = [&](const char* k, double def) -> double {
    return p[k] ? p[k].as<double>() : def;
  };
  auto get_z = [&](const char* k, size_t def) -> size_t {
    return p[k] ? p[k].as<size_t>() : def;
  };
  auto get_b = [&](const char* k, bool def) -> bool {
    return p[k] ? p[k].as<bool>() : def;
  };

  auto& cfg = bc.odom;

  bc.min_range = get_d("min_range", 1.0);
  bc.max_range = get_d("max_range", 30.0);
  bc.min_intensity = get_z("min_intensity", 0);

  cfg.kdtree.max_points_per_leaf = get_z("kdtree.max_points_per_leaf", 20);
  cfg.kdtree.max_depth = get_z("kdtree.max_depth", 16);
  cfg.kdtree.max_parallel_level = get_z("kdtree.max_parallel_level", 3);
  cfg.kdtree.leaf_extent_sq = get_d("kdtree.leaf_extent_sq", 0.04);

  cfg.icp.n_iters = get_z("icp.n_iters", 10);
  cfg.icp.max_correspondence_dist = get_d("icp.max_correspondence_dist", 1.0);
  cfg.icp.huber_delta = get_d("icp.huber_delta", 0.1);
  cfg.icp.damping = get_d("icp.damping", 0.0);

  cfg.scan_dt = get_d("scan_dt", 0.1);
  cfg.kf_ratio = get_d("keyframe_min_ratio", 0.3);
  cfg.max_keyframes = get_z("max_keyframes", 4);

  auto& ekf = cfg.ekf;
  ekf.accel_noise = get_d("ekf.accel_noise", 0.1);
  ekf.gyro_noise = get_d("ekf.gyro_noise", 0.01);
  ekf.accel_bias_rw = get_d("ekf.accel_bias_rw", 1e-4);
  ekf.gyro_bias_rw = get_d("ekf.gyro_bias_rw", 1e-5);
  ekf.calib_samples = get_z("ekf.calib_samples", 200);
  ekf.still_window = get_d("ekf.still_window", 0.5);
  ekf.still_accel_var = get_d("ekf.still_accel_var", 0.01);
  ekf.still_gyro_var = get_d("ekf.still_gyro_var", 1e-4);
  ekf.still_accel_thr = get_d("ekf.still_accel_thr", 0.5);
  ekf.still_gyro_thr = get_d("ekf.still_gyro_thr", 0.05);
  ekf.do_zupt = get_b("ekf.do_zupt", false);
  ekf.zupt_noise = get_d("ekf.zupt_noise", 1e-4);
  ekf.init_pos_var = get_d("ekf.init_pos_var", 1e-3);
  ekf.init_vel_var = get_d("ekf.init_vel_var", 1e-3);
  ekf.init_rot_var = get_d("ekf.init_rot_var", 1e-3);
  ekf.init_accel_bias_var = get_d("ekf.init_accel_bias_var", 1e-3);
  ekf.init_gyro_bias_var = get_d("ekf.init_gyro_bias_var", 1e-6);

  if (p["ekf.gravity"] && p["ekf.gravity"].IsSequence()) {
    auto g = p["ekf.gravity"].as<std::vector<double>>();
    if (g.size() == 3) ekf.gravity = fodom::Vec3(g[0], g[1], g[2]);
  }

  if (p["imu_in_lidar"] && p["imu_in_lidar"].IsSequence()) {
    auto m = p["imu_in_lidar"].as<std::vector<double>>();
    if (m.size() == 16) {
      Eigen::Matrix4d mat;
      for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) mat(r, c) = m[r * 4 + c];
      cfg.imu_in_lidar.matrix() = mat;
    }
  }

  return bc;
}

// ---- main ------------------------------------------------------------------

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <bag_path> <output.tum> [<config.yaml>]"
                 " [--imu TOPIC] [--points TOPIC]"
                 " [--min-range M] [--max-range M] [--min-intensity N]\n";
    return 1;
  }

  std::string bag_path = argv[1];
  std::string out_path = argv[2];
  std::string cfg_path;
  std::string imu_topic = "/ouster/imu";
  std::string pc_topic = "/ouster/points";

  // Optional overrides parsed before config so they can shadow YAML values.
  double cli_min_range = -1.0, cli_max_range = -1.0;
  int cli_min_intensity = -1;

  for (int i = 3; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--imu" && i + 1 < argc)
      imu_topic = argv[++i];
    else if (a == "--points" && i + 1 < argc)
      pc_topic = argv[++i];
    else if (a == "--min-range" && i + 1 < argc)
      cli_min_range = std::stod(argv[++i]);
    else if (a == "--max-range" && i + 1 < argc)
      cli_max_range = std::stod(argv[++i]);
    else if (a == "--min-intensity" && i + 1 < argc)
      cli_min_intensity = std::stoi(argv[++i]);
    else if (a[0] != '-' && cfg_path.empty())
      cfg_path = a;
    else
      std::cerr << "Warning: unknown argument '" << a << "'\n";
  }

  BagConfig bc = load_yaml(cfg_path);
  if (cli_min_range >= 0.0) bc.min_range = cli_min_range;
  if (cli_max_range >= 0.0) bc.max_range = cli_max_range;
  if (cli_min_intensity >= 0)
    bc.min_intensity = static_cast<size_t>(cli_min_intensity);

  fodom::Odom odom;
  odom.config =
      bc.odom;  // ekf.config is a ref into odom.config.ekf — updated in place

  // Open bag
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_opts;
  storage_opts.uri = bag_path;
  rosbag2_cpp::ConverterOptions conv_opts;
  conv_opts.input_serialization_format = "cdr";
  conv_opts.output_serialization_format = "cdr";

  try {
    reader.open(storage_opts, conv_opts);
  } catch (const std::exception& e) {
    std::cerr << "Failed to open bag '" << bag_path << "': " << e.what()
              << "\n";
    return 1;
  }

  rosbag2_storage::StorageFilter filter;
  filter.topics = {imu_topic, pc_topic};
  reader.set_filter(filter);

  rclcpp::init(0, nullptr);

  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_ser;

  std::ofstream tum(out_path);
  if (!tum) {
    std::cerr << "Cannot open output file '" << out_path << "'\n";
    return 1;
  }
  tum << std::fixed << std::setprecision(9);
  tum << "# timestamp tx ty tz qx qy qz qw\n";

  fodom::PCloudType cloud;
  const double min_r2 = bc.min_range * bc.min_range;
  const double max_r2 = bc.max_range * bc.max_range;

  size_t n_imu = 0, n_scans = 0, n_poses = 0;

  while (reader.has_next()) {
    auto msg = reader.read_next();

    if (msg->topic_name == imu_topic) {
      rclcpp::SerializedMessage ser(*msg->serialized_data);
      auto m = std::make_shared<sensor_msgs::msg::Imu>();
      imu_ser.deserialize_message(&ser, m.get());

      fodom::ImuMeasurement imu;
      imu.accel =
          fodom::Vec3(m->linear_acceleration.x, m->linear_acceleration.y,
                      m->linear_acceleration.z);
      imu.gyro = fodom::Vec3(m->angular_velocity.x, m->angular_velocity.y,
                             m->angular_velocity.z);
      odom.feed_imu(rclcpp::Time(m->header.stamp).seconds(), imu);
      ++n_imu;

    } else if (msg->topic_name == pc_topic) {
      rclcpp::SerializedMessage ser(*msg->serialized_data);
      auto m = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pc_ser.deserialize_message(&ser, m.get());

      const size_t n = m->width * m->height;
      cloud.clear();
      cloud.reserve(n);

      sensor_msgs::PointCloud2ConstIterator<float> x_it(*m, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_it(*m, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_it(*m, "z");
      sensor_msgs::PointCloud2ConstIterator<uint32_t> i_it(*m, "intensity");
      sensor_msgs::PointCloud2ConstIterator<uint32_t> t_it(*m, "t");

      for (size_t i = 0; i < n; ++i, ++x_it, ++y_it, ++z_it, ++i_it, ++t_it) {
        if (!std::isfinite(*x_it) || !std::isfinite(*y_it) ||
            !std::isfinite(*z_it))
          continue;
        const fodom::Vec3 pt(*x_it, *y_it, *z_it);
        const double r2 = pt.squaredNorm();
        if (r2 < min_r2 || r2 > max_r2) continue;
        if (*i_it < bc.min_intensity) continue;
        cloud.push_back({pt, *t_it});
      }

      const double stamp = rclcpp::Time(m->header.stamp).seconds();
      odom.feed_cloud(stamp, cloud);
      ++n_scans;

      if (odom.keyframes.empty()) continue;

      const fodom::PoseType pose =
          odom.ekf.pose() * odom.config.imu_in_lidar.inverse();
      const Eigen::Quaterniond q(pose.rotation());
      const Eigen::Vector3d t = pose.translation();

      tum << stamp << " " << t.x() << " " << t.y() << " " << t.z() << " "
          << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
      ++n_poses;

      if (n_scans % 50 == 0)
        std::cerr << "\r" << n_scans << " scans processed, " << n_poses
                  << " poses written..." << std::flush;
    }
  }

  std::cerr << "\nDone: " << n_imu << " IMU, " << n_scans << " scans, "
            << n_poses << " poses -> " << out_path << "\n";

  rclcpp::shutdown();
  return 0;
}
