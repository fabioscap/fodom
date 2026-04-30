#include "fodom/odom.h"

#include <iostream>

namespace fodom {

void Odom::process_imu(double stamp, const ImuMeasurement& imu) {
  imu_buf.push_back({stamp, imu});
  ekf.update_stillness(imu, stamp);

  if (ekf.status == EkfStatus::Calibrating) {
    if (ekf.is_stationary) {
      ekf.calib_accels.push_back(imu.accel);
      ekf.calib_gyros.push_back(imu.gyro);
    } else if (!ekf.calib_accels.empty()) {
      std::cerr << "[fodom] IMU moved during calibration, initializing from "
                << ekf.calib_accels.size() << " samples\n";
      ekf.init_from_calib();
      ekf.calib_accels.clear();
      ekf.calib_gyros.clear();
      ekf.last_stamp = stamp;
    }

    if (ekf.status == EkfStatus::Calibrating &&
        ekf.calib_accels.size() >= config.ekf.calib_samples) {
      ekf.init_from_calib();
      ekf.calib_accels.clear();
      ekf.calib_gyros.clear();
      ekf.last_stamp = stamp;
    }
    return;
  }

  ekf.predict(imu, stamp);
}

void Odom::process_cloud(double stamp, const PCloudType& cloud) {
  // TODO: deskew cloud using imu_buf, then ICP
}

}  // namespace fodom
