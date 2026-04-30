#include "fodom/odom.h"

#include <iostream>

namespace fodom {

void Odom::feed_imu(double stamp, const ImuMeasurement& imu) {
  imu_buf.push_back({stamp, imu});
}

void Odom::feed_cloud(double stamp, const PCloudType& cloud) {
  traj.clear();
  size_t n_integrated = 0;
  traj.emplace_back(ekf.pose());
  while (!imu_buf.empty() and stamp >= imu_buf.front().first) {
    ekf.step(imu_buf.front().first, imu_buf.front().second);

    imu_buf.pop_front();
    ++n_integrated;
  }

  std::cerr << "Integrated " << n_integrated << " IMU messages\n";
}

}  // namespace fodom
