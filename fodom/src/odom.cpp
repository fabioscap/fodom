#include "fodom/odom.h"

#include <chrono>
#include <iostream>
#include <thread>

namespace fodom {

using Clock = std::chrono::high_resolution_clock;
static double ms_since(Clock::time_point t) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

void Odom::feed_imu(double stamp, const ImuMeasurement& imu) {
  imu_buf.push_back({stamp, imu});
}

void Odom::feed_cloud(double stamp, PCloudType& cloud) {
  traj.clear();
  if (last_scan_stamp >= 0) traj.emplace_back(last_scan_stamp, ekf.pose());

  const double scan_end_time = stamp + config.scan_dt;
  while (!imu_buf.empty() && imu_buf.front().first <= scan_end_time) {
    const auto [imu_stamp, imu] = imu_buf.front();
    ekf.step(imu_stamp, imu);
    traj.emplace_back(imu_stamp, ekf.pose());
    imu_buf.pop_front();
  }
  last_scan_stamp = traj.empty() ? stamp : traj.back().first;

  if (ekf.status == EkfStatus::Calibrating) return;

  // cloud is now in IMU frame for the remainder of this function
  transform_cloud(cloud, config.imu_in_lidar.inverse());
  const auto t_deskew = Clock::now();
  if (!traj.empty()) deskew_cloud(cloud, stamp, traj);
  const double dt_deskew = ms_since(t_deskew);

  const auto t_tree = Clock::now();
  auto tree =
      std::make_unique<TreeNode>(cloud.begin(), cloud.end(), config.kdtree);
  const double dt_tree = ms_since(t_tree);

  if (keyframes.empty()) {
    keyframes.push_back({std::move(tree), ekf.pose()});
    return;
  }

  const auto t_leaves = Clock::now();
  std::vector<Vec3> leaves;
  leaves.reserve(cloud.size());
  get_leaves(*tree, leaves);
  const double dt_leaves = ms_since(t_leaves);

  const Scalar max_dist_sq =
      config.icp.max_correspondence_dist * config.icp.max_correspondence_dist;
  const Scalar inv_var =
      1.0 / (ekf.config.point_noise * ekf.config.point_noise);
  const Scalar huber_delta = config.icp.huber_delta;
  const int max_pts = static_cast<int>(leaves.size() * keyframes.size());

  const auto t_icp = Clock::now();
  IcpContrib total;
  for (int iter = 0; iter < static_cast<int>(config.icp.n_iters); ++iter) {
    const PoseType curr_imu_in_world = ekf.pose();

    std::vector<IcpContrib> contribs(keyframes.size());
    std::vector<std::thread> threads;
    threads.reserve(keyframes.size());
    for (size_t k = 0; k < keyframes.size(); ++k) {
      threads.emplace_back([&, k, curr_imu_in_world]() {
        contribs[k] = accumulate_keyframe(*keyframes[k].tree, keyframes[k].pose,
                                          leaves, curr_imu_in_world, inv_var,
                                          max_dist_sq, huber_delta);
      });
    }
    for (auto& t : threads) t.join();

    total = {};
    for (const auto& c : contribs) {
      total.H += c.H;
      total.b += c.b;
      total.valid += c.valid;
    }

    if (total.valid == 0) break;
    const bool is_last = (iter == static_cast<int>(config.icp.n_iters) - 1);
    ekf.update_points(total.H, total.b, is_last);
  }
  const double dt_icp = ms_since(t_icp);

  const Scalar ratio =
      static_cast<Scalar>(total.valid) / static_cast<Scalar>(max_pts);
  if (ratio < config.kf_ratio) {
    keyframes.push_back({std::move(tree), ekf.pose()});
    if (keyframes.size() > config.max_keyframes) keyframes.pop_front();
    std::cerr << "Keyframe! (" << keyframes.size() << "/"
              << config.max_keyframes << ")\n";
  }

  std::cerr << "deskew: " << dt_deskew << " ms | tree: " << dt_tree
            << " ms | leaves: " << dt_leaves << " ms | icp: " << dt_icp
            << " ms\n";
  std::cerr << "valid: " << total.valid << "/" << max_pts
            << "  ratio: " << ratio << "\n";
  std::cerr << "velocity: " << ekf.state.velocity.norm()
            << "  gyro_bias: " << ekf.state.gyro_bias.norm()
            << "  accel_bias: " << ekf.state.accel_bias.norm() << "\n";
}

}  // namespace fodom
