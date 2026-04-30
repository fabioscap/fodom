#pragma once

#include <fodom/imu.h>
#include <fodom/kdtree.h>

#include <deque>
#include <utility>

namespace fodom {

struct OdomConfig {
  KdTreeConfig kdtree;
  EkfConfig ekf;
  ICPConfig icp;
};

class Odom {
 public:
  Odom() : ekf(config.ekf) { traj.reserve(2 * freq_ratio); }
  //
  void feed_imu(double stamp, const ImuMeasurement& imu);
  void feed_cloud(double stamp, const PCloudType& cloud);

  OdomConfig config;
  Ekf ekf;

  std::deque<std::pair<double, ImuMeasurement>> imu_buf;

  // trajectory in between two scans
  std::vector<PoseType> traj;

  // IMU in LiDAR extrinsic matrix
  PoseType imu_in_lidar = PoseType::Identity();

  size_t freq_ratio = 10;  // how many imu packets per lidar cloud, in general
};

}  // namespace fodom
