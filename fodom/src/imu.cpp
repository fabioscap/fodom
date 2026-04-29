#include "fodom/imu.h"

#include <iostream>

namespace fodom {

using Mat6 = Eigen::Matrix<Scalar, 6, 6>;
using Mat12 = Eigen::Matrix<Scalar, 12, 12>;
using Mat15 = Eigen::Matrix<Scalar, 15, 15>;
using Vec6 = Eigen::Matrix<Scalar, 6, 1>;
using Vec15 = Eigen::Matrix<Scalar, 15, 1>;
using Mat6x15 = Eigen::Matrix<Scalar, 6, 15>;
using Mat15x6 = Eigen::Matrix<Scalar, 15, 6>;
using Mat15x12 = Eigen::Matrix<Scalar, 15, 12>;

void Ekf::init_from_calib() {
  const Scalar n = static_cast<Scalar>(calib_accels.size());

  Vec3 mean_accel = Vec3::Zero();
  Vec3 mean_gyro = Vec3::Zero();
  for (const auto& a : calib_accels) mean_accel += a;
  for (const auto& g : calib_gyros) mean_gyro += g;
  mean_accel /= n;
  mean_gyro /= n;

  state.gyro_bias = mean_gyro;
  state.rotation = Eigen::Quaterniond::FromTwoVectors(
                       mean_accel.normalized(), (-config.gravity).normalized())
                       .toRotationMatrix();
  status = EkfStatus::Running;
}

void Ekf::update_stillness(const ImuMeasurement& imu) {
  still_buf.push_back(imu);
  still_sum_a += imu.accel;
  still_sum_g += imu.gyro;
  still_sumsq_a += imu.accel.squaredNorm();
  still_sumsq_g += imu.gyro.squaredNorm();

  while (imu.stamp - still_buf.front().stamp > config.still_window) {
    const auto& old = still_buf.front();
    still_sum_a -= old.accel;
    still_sum_g -= old.gyro;
    still_sumsq_a -= old.accel.squaredNorm();
    still_sumsq_g -= old.gyro.squaredNorm();
    still_buf.pop_front();
  }

  if (still_buf.size() >= 2) {
    const Scalar n = static_cast<Scalar>(still_buf.size());
    const Vec3 mean_a = still_sum_a / n;
    const Vec3 mean_g = still_sum_g / n;
    const Scalar var_a = still_sumsq_a / n - mean_a.squaredNorm();
    const Scalar var_g = still_sumsq_g / n - mean_g.squaredNorm();

    const Scalar g_mag = config.gravity.norm();
    is_stationary = var_a < config.still_accel_var &&
                    var_g < config.still_gyro_var &&
                    std::abs(mean_a.norm() - g_mag) < config.still_accel_thr &&
                    mean_g.norm() < config.still_gyro_thr;
  } else {
    is_stationary = false;
  }
}

void Ekf::process_imu(const ImuMeasurement& imu) {
  update_stillness(imu);

  if (status == EkfStatus::Calibrating) {
    if (is_stationary) {
      calib_accels.push_back(imu.accel);
      calib_gyros.push_back(imu.gyro);
    } else if (!calib_accels.empty()) {
      std::cerr << "[fodom] IMU moved during calibration, initializing from "
                << calib_accels.size() << " samples\n";
      init_from_calib();
      calib_accels.clear();
      calib_gyros.clear();
      last_stamp = imu.stamp;
    }

    if (status == EkfStatus::Calibrating &&
        calib_accels.size() >= config.calib_samples) {
      init_from_calib();
      calib_accels.clear();
      calib_gyros.clear();
      last_stamp = imu.stamp;
    }
    return;
  }

  predict(imu);
}

void Ekf::predict(const ImuMeasurement& imu) {
  if (last_stamp < 0.0) {
    last_stamp = imu.stamp;
    return;
  }

  const Scalar dt = static_cast<Scalar>(imu.stamp - last_stamp);
  last_stamp = imu.stamp;

  const Vec3 a = imu.accel - state.accel_bias;
  const Vec3 w = imu.gyro - state.gyro_bias;

  const Vec3 a_world = state.rotation * a + config.gravity;

  state.position += state.velocity * dt + 0.5 * a_world * dt * dt;
  state.velocity += a_world * dt;
  state.rotation = normalize_so3(state.rotation * exp_so3(w * dt));

  Mat15 F = Mat15::Zero();
  F.block<3, 3>(0, 3) = Mat3::Identity();
  F.block<3, 3>(3, 6) = -state.rotation * skew(a);
  F.block<3, 3>(3, 9) = -state.rotation;
  F.block<3, 3>(6, 6) = -skew(w);
  F.block<3, 3>(6, 12) = -Mat3::Identity();

  const Mat15 Phi = Mat15::Identity() + F * dt;

  Mat15x12 G = Mat15x12::Zero();
  G.block<3, 3>(3, 0) = -state.rotation;
  G.block<3, 3>(6, 3) = -Mat3::Identity();
  G.block<3, 3>(9, 6) = Mat3::Identity();
  G.block<3, 3>(12, 9) = Mat3::Identity();

  Mat12 Qc = Mat12::Zero();
  Qc.block<3, 3>(0, 0) =
      (config.accel_noise * config.accel_noise) * Mat3::Identity();
  Qc.block<3, 3>(3, 3) =
      (config.gyro_noise * config.gyro_noise) * Mat3::Identity();
  Qc.block<3, 3>(6, 6) =
      (config.accel_bias_rw * config.accel_bias_rw) * Mat3::Identity();
  Qc.block<3, 3>(9, 9) =
      (config.gyro_bias_rw * config.gyro_bias_rw) * Mat3::Identity();

  state.P = Phi * state.P * Phi.transpose() + G * Qc * G.transpose() * dt;
}

void Ekf::update_pose(const PoseType& pose) {
  if (status != EkfStatus::Running) return;

  Mat6x15 H = Mat6x15::Zero();
  H.block<3, 3>(0, 0) = Mat3::Identity();
  H.block<3, 3>(3, 6) = Mat3::Identity();

  Mat6 R_meas = Mat6::Zero();
  R_meas.block<3, 3>(0, 0) =
      (config.pos_noise * config.pos_noise) * Mat3::Identity();
  R_meas.block<3, 3>(3, 3) =
      (config.rot_noise * config.rot_noise) * Mat3::Identity();

  Vec6 z;
  z.head<3>() = pose.translation() - state.position;
  z.tail<3>() = log_so3(state.rotation.transpose() * pose.linear());

  const Mat6 S = H * state.P * H.transpose() + R_meas;
  const Mat15x6 K = state.P * H.transpose() * S.inverse();

  const Vec15 dx = K * z;
  state.position += dx.segment<3>(0);
  state.velocity += dx.segment<3>(3);
  state.rotation = normalize_so3(state.rotation * exp_so3(dx.segment<3>(6)));
  state.accel_bias += dx.segment<3>(9);
  state.gyro_bias += dx.segment<3>(12);

  const Mat15 I_KH = Mat15::Identity() - K * H;
  state.P = I_KH * state.P * I_KH.transpose() + K * R_meas * K.transpose();
}

PoseType Ekf::pose() const {
  PoseType p = PoseType::Identity();
  p.linear() = state.rotation;
  p.translation() = state.position;
  return p;
}

}  // namespace fodom
