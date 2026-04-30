#pragma once

#include <deque>
#include <utility>

#include <fodom/imu.h>
#include <fodom/kdtree.h>

namespace fodom {

struct OdomConfig {
  KdTreeConfig kdtree;
  EkfConfig ekf;
  ICPConfig icp;
};

struct Odom {
  Odom() : ekf(config.ekf) {}

  void process_imu(double stamp, const ImuMeasurement& imu);
  void process_cloud(double stamp, const PCloudType& cloud);

  OdomConfig config;
  Ekf ekf;

  std::deque<std::pair<double, ImuMeasurement>> imu_buf;
};

}  // namespace fodom
