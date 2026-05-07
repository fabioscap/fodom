#pragma once

#include <fodom/imu.h>
#include <fodom/kdtree.h>

#include <deque>
#include <utility>

namespace fodom {

struct OdomConfig {
  Scalar kf_ratio = 0.4;
  Scalar scan_dt = 0.1;
  PoseType imu_in_lidar;
  size_t freq_ratio = 10;
  size_t max_keyframes = 4;
  KdTreeConfig kdtree;
  EkfConfig ekf;
  ICPConfig icp;
};

struct Keyframe {
  std::unique_ptr<TreeNode> tree;
  PoseType pose;
};

class Odom {
 public:
  Odom() : ekf(config.ekf) { traj.reserve(2 * config.freq_ratio); }

  void feed_imu(double stamp, const ImuMeasurement& imu);
  void feed_cloud(double stamp, PCloudType& cloud);

  OdomConfig config;
  Ekf ekf;

  std::deque<std::pair<double, ImuMeasurement>> imu_buf;
  std::vector<std::pair<double, PoseType>> traj;
  double last_scan_stamp = -1.0;

  std::deque<Keyframe> keyframes;
};

}  // namespace fodom
