#include "fodom/odom.h"

#include <algorithm>
#include <iostream>

namespace fodom {

void Odom::feed_imu(double stamp, const ImuMeasurement& imu) {
  imu_buf.push_back({stamp, imu});
}

static PoseType interp_pose(const std::vector<std::pair<double, PoseType>>& traj,
                            double t) {
  if (t <= traj.front().first) return traj.front().second;
  if (t >= traj.back().first) return traj.back().second;
  auto it = std::lower_bound(traj.begin(), traj.end(), t,
                             [](const auto& e, double v) { return e.first < v; });
  const auto& [t1, P1] = *it;
  const auto& [t0, P0] = *std::prev(it);
  const double a = (t - t0) / (t1 - t0);
  PoseType out;
  out.translation() = P0.translation() + a * (P1.translation() - P0.translation());
  out.linear() = Eigen::Quaterniond(P0.rotation())
                     .slerp(a, Eigen::Quaterniond(P1.rotation()))
                     .toRotationMatrix();
  return out;
}

void Odom::feed_cloud(double stamp, PCloudType& cloud) {
  traj.clear();
  if (last_scan_stamp >= 0) traj.emplace_back(last_scan_stamp, ekf.pose());

  while (!imu_buf.empty() && imu_buf.front().first <= stamp) {
    const auto [imu_stamp, imu] = imu_buf.front();
    ekf.step(imu_stamp, imu);
    traj.emplace_back(imu_stamp, ekf.pose());
    imu_buf.pop_front();
  }
  last_scan_stamp = stamp;

  if (ekf.status == EkfStatus::Calibrating) return;

  // deskew: transform each point to the scan-start reference frame
  if (traj.size() >= 2) {
    const PoseType T_I_L = config.imu_in_lidar.inverse();
    const PoseType T_W_L_ref = interp_pose(traj, stamp) * T_I_L;
    for (auto& p : cloud) {
      const double t_abs = stamp + static_cast<double>(p.nsec) * 1e-9;
      const PoseType T_deskew = T_W_L_ref.inverse() * interp_pose(traj, t_abs) * T_I_L;
      p.xyz = T_deskew * p.xyz;
    }
  }

  if (!keyframe) {
    keyframe =
        std::make_unique<TreeNode>(cloud.begin(), cloud.end(), config.kdtree);
    keyframe_pose = ekf.pose();
    return;
  } else {
    auto tree =
        std::make_unique<TreeNode>(cloud.begin(), cloud.end(), config.kdtree);
    std::vector<Vec3> leaves;
    get_leaves(*tree, leaves);

    const PoseType lidar_in_imu = config.imu_in_lidar.inverse();

    const PoseType lidar_kf_in_world = keyframe_pose * lidar_in_imu;
    const PoseType lidar_curr_in_world = ekf.pose() * lidar_in_imu;
    const PoseType init_guess =
        lidar_kf_in_world.inverse() * lidar_curr_in_world;

    auto result = match_kdtree(*keyframe, leaves, init_guess, config.icp);

    PoseType& T_icp = result.pose;

    const PoseType updated_pose =
        keyframe_pose * lidar_in_imu * T_icp * config.imu_in_lidar;

    if (result.ratio < config.kf_ratio) {
      std::cerr << "Keyframe!\n";
      // spawn new keyframe
      keyframe = std::move(tree);
      keyframe_pose = updated_pose;
    }

    ekf.update_pose(updated_pose, stamp);
  }
}

}  // namespace fodom
