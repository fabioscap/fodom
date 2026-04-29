#pragma once

#include <deque>
#include <vector>

#include "fodom/math.h"

namespace fodom {

enum class EkfStatus { Calibrating, Running };

struct ImuMeasurement {
  Vec3 accel;
  Vec3 gyro;
  double stamp;  // seconds
};

struct EkfConfig {
  Scalar accel_noise = 0.1;
  Scalar gyro_noise = 0.01;
  Scalar accel_bias_rw = 1e-4;
  Scalar gyro_bias_rw = 1e-5;
  Scalar pos_noise = 0.05;
  Scalar rot_noise = 0.01;
  Vec3 gravity{0.0, 0.0, -9.81};

  size_t calib_samples = 200;

  Scalar still_window = 0.5;      // [s]    time window for variance computation
  Scalar still_accel_var = 0.01;  // [m²/s⁴]  max accel variance (trace of cov)
  Scalar still_gyro_var = 1e-4;   // [rad²/s²] max gyro variance
  Scalar still_accel_thr = 0.5;   // [m/s²]  max deviation of mean |a| from |g|
  Scalar still_gyro_thr = 0.05;   // [rad/s] max mean gyro magnitude
};

struct EkfState {
  Vec3 position = Vec3::Zero();
  Vec3 velocity = Vec3::Zero();
  Mat3 rotation = Mat3::Identity();
  Vec3 accel_bias = Vec3::Zero();
  Vec3 gyro_bias = Vec3::Zero();
  Eigen::Matrix<Scalar, 15, 15> P = Eigen::Matrix<Scalar, 15, 15>::Identity();
};

struct Ekf {
  Ekf(EkfConfig& cfg) : config(cfg) {}

  void process_imu(const ImuMeasurement& imu);
  void update_stillness(const ImuMeasurement& imu);
  void predict(const ImuMeasurement& imu);
  void update_pose(const PoseType& pose);
  void init_from_calib();
  PoseType pose() const;

  EkfState state;
  EkfConfig& config;
  EkfStatus status = EkfStatus::Calibrating;
  bool is_stationary = false;
  double last_stamp = -1.0;

  std::vector<Vec3> calib_accels;
  std::vector<Vec3> calib_gyros;
  std::deque<ImuMeasurement> still_buf;

  Vec3 still_sum_a = Vec3::Zero();
  Vec3 still_sum_g = Vec3::Zero();
  Scalar still_sumsq_a = 0.0;
  Scalar still_sumsq_g = 0.0;
};

}  // namespace fodom
