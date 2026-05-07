#include "fodom/odom.h"

#include <algorithm>
#include <thread>
#include <iostream>

namespace fodom {

using Mat15 = Eigen::Matrix<Scalar, 15, 15>;
using Vec15 = Eigen::Matrix<Scalar, 15, 1>;

void Odom::feed_imu(double stamp, const ImuMeasurement& imu) {
  imu_buf.push_back({stamp, imu});
}

static PoseType interp_pose(
    const std::vector<std::pair<double, PoseType>>& traj, double t) {
  if (t <= traj.front().first) return traj.front().second;
  if (t >= traj.back().first) return traj.back().second;
  auto it =
      std::lower_bound(traj.begin(), traj.end(), t,
                       [](const auto& e, double v) { return e.first < v; });
  const auto& [t1, P1] = *it;
  const auto& [t0, P0] = *std::prev(it);
  const double a = (t - t0) / (t1 - t0);
  PoseType out;
  out.translation() =
      P0.translation() + a * (P1.translation() - P0.translation());
  out.linear() = Eigen::Quaterniond(P0.rotation())
                     .slerp(a, Eigen::Quaterniond(P1.rotation()))
                     .toRotationMatrix();
  return out;
}

struct KfContrib {
  Mat15 H = Mat15::Zero();
  Vec15 b = Vec15::Zero();
  int valid = 0;
};

static KfContrib accumulate_keyframe(const TreeNode& kf_tree,
                                     const PoseType& kf_lidar_in_world,
                                     const std::vector<Vec3>& leaves,
                                     const PoseType& curr_lidar_in_world,
                                     const Vec3& imu_pos, const Mat3& imu_rot,
                                     Scalar inv_var, Scalar max_dist_sq,
                                     Scalar huber_delta) {
  const PoseType curr_to_kf = kf_lidar_in_world.inverse() * curr_lidar_in_world;

  KfContrib result;
  for (int i = 0; i < static_cast<int>(leaves.size()); ++i) {
    const Vec3& point = leaves[i];
    const Vec3 point_in_kf = curr_to_kf * point;
    const TreeNode& matched = kf_tree.nn_plane(point_in_kf);
    if ((point_in_kf - matched.mean).squaredNorm() > max_dist_sq) continue;

    const Vec3 n_world = kf_lidar_in_world.linear() * matched.normal;
    const Vec3 mu_world = kf_lidar_in_world * matched.mean;
    const Vec3 p_world = curr_lidar_in_world * point;

    Eigen::Matrix<Scalar, 1, 15> J;
    J.block<1, 3>(0, 0) = n_world.transpose();
    J.block<1, 3>(0, 3).setZero();
    J.block<1, 3>(0, 6) =
        -n_world.transpose() * skew(p_world - imu_pos) * imu_rot;
    J.block<1, 6>(0, 9).setZero();
    const Scalar z = n_world.dot(mu_world - p_world);

    const Scalar cos_inc =
        std::abs(n_world.dot((p_world - imu_pos).normalized()));
    const Scalar range_sq = point.squaredNorm();
    const Scalar abs_z = std::abs(z);
    const Scalar w_huber = (abs_z <= huber_delta) ? 1.0 : huber_delta / abs_z;
    const Scalar w = inv_var * cos_inc / (range_sq + 1e-6) * w_huber;

    result.H += w * J.transpose() * J;
    result.b += w * J.transpose() * z;
    ++result.valid;
  }
  return result;
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

  if (traj.size() >= 2) {
    const PoseType T_I_L = config.imu_in_lidar.inverse();
    const PoseType T_W_L_ref = traj.back().second * T_I_L;
    for (auto& p : cloud) {
      const double t_abs = stamp + static_cast<double>(p.nsec) * 1e-9;
      const PoseType T_deskew =
          T_W_L_ref.inverse() * interp_pose(traj, t_abs) * T_I_L;
      p.xyz = T_deskew * p.xyz;
    }
  }

  auto tree =
      std::make_unique<TreeNode>(cloud.begin(), cloud.end(), config.kdtree);

  if (keyframes.empty()) {
    keyframes.push_back({std::move(tree), ekf.pose()});
    return;
  }

  std::vector<Vec3> leaves;
  get_leaves(*tree, leaves);

  const PoseType lidar_in_imu = config.imu_in_lidar.inverse();
  const Scalar max_dist_sq =
      config.icp.max_correspondence_dist * config.icp.max_correspondence_dist;
  const Scalar inv_var =
      1.0 / (ekf.config.point_noise * ekf.config.point_noise);
  const Scalar huber_delta = config.icp.huber_delta;
  const int max_pts = static_cast<int>(leaves.size() * keyframes.size());

  int valid = 0;
  for (int iter = 0; iter < static_cast<int>(config.icp.n_iters); ++iter) {
    const PoseType lidar_curr_in_world = ekf.pose() * lidar_in_imu;

    std::vector<KfContrib> contribs(keyframes.size());
    std::vector<std::thread> threads;
    threads.reserve(keyframes.size());
    for (size_t k = 0; k < keyframes.size(); ++k) {
      const PoseType kf_lidar_in_world = keyframes[k].pose * lidar_in_imu;
      threads.emplace_back([&, k, kf_lidar_in_world]() {
        contribs[k] = accumulate_keyframe(
            *keyframes[k].tree, kf_lidar_in_world, leaves, lidar_curr_in_world,
            ekf.state.position, ekf.state.rotation, inv_var, max_dist_sq,
            huber_delta);
      });
    }
    for (auto& t : threads) t.join();

    Mat15 H = Mat15::Zero();
    Vec15 b = Vec15::Zero();
    valid = 0;
    for (const auto& c : contribs) { H += c.H; b += c.b; valid += c.valid; }

    if (valid == 0) break;
    const bool is_last = (iter == static_cast<int>(config.icp.n_iters) - 1);
    ekf.update_points(H, b, is_last);
  }

  const Scalar ratio =
      static_cast<Scalar>(valid) / static_cast<Scalar>(max_pts);
  if (ratio < config.kf_ratio) {
    keyframes.push_back({std::move(tree), ekf.pose()});
    if (keyframes.size() > config.max_keyframes) keyframes.pop_front();
    std::cerr << "Keyframe! (" << keyframes.size() << "/"
              << config.max_keyframes << ")\n";
  }

  std::cerr << "valid: " << valid << "/" << max_pts << "  ratio: " << ratio
            << "\n";
  std::cerr << "velocity: " << ekf.state.velocity.norm()
            << "  gyro_bias: " << ekf.state.gyro_bias.norm()
            << "  accel_bias: " << ekf.state.accel_bias.norm() << "\n";
}

}  // namespace fodom
