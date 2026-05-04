#pragma once

#include <fodom/imu.h>
#include <fodom/kdtree.h>

#include <deque>
#include <utility>

namespace fodom {

struct OdomConfig {
  Scalar kf_ratio =
      0.4;  // ratio of matched leaves under which a new keyframe is spawned
  PoseType imu_in_lidar;

  size_t freq_ratio = 10;  // how many imu packets per lidar cloud, in general

  KdTreeConfig kdtree;
  EkfConfig ekf;
  ICPConfig icp;
};

class Odom {
 public:
  Odom() : ekf(config.ekf) { traj.reserve(2 * config.freq_ratio); }
  //
  void feed_imu(double stamp, const ImuMeasurement& imu);
  void feed_cloud(double stamp, PCloudType& cloud);

  OdomConfig config;
  Ekf ekf;

  std::deque<std::pair<double, ImuMeasurement>> imu_buf;

  // trajectory in between two scans: (stamp, imu_pose_in_world)
  std::vector<std::pair<double, PoseType>> traj;

  double last_scan_stamp = -1.0;

  // LiDAR odometry
  std::unique_ptr<TreeNode> keyframe;
  PoseType keyframe_pose = PoseType::Identity();
};

}  // namespace fodom
