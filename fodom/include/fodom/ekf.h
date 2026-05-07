#pragma once

#include <deque>
#include <utility>
#include <vector>

#include "fodom/kdtree.h"
#include "fodom/math.h"

namespace fodom {

enum class EkfStatus { Calibrating, Running };

struct ImuMeasurement {
  Vec3 accel;
  Vec3 gyro;
};

struct EkfConfig {
  Scalar accel_noise = 0.1;
  Scalar gyro_noise = 0.01;
  Scalar accel_bias_rw = 1e-4;
  Scalar gyro_bias_rw = 1e-5;
  Scalar pos_noise = 0.05;
  Scalar rot_noise = 0.01;
  Scalar point_noise = 0.02;
  Vec3 gravity{0.0, 0.0, -9.81};

  size_t calib_samples = 200;

  Scalar still_window = 0.5;
  Scalar still_accel_var = 0.01;
  Scalar still_gyro_var = 1e-4;
  Scalar still_accel_thr = 0.5;
  Scalar still_gyro_thr = 0.05;

  bool do_zupt = false;
  Scalar zupt_noise = 1e-4;

  Scalar init_pos_var = 1e-3;
  Scalar init_vel_var = 1e-3;
  Scalar init_rot_var = 1e-3;
  Scalar init_accel_bias_var = 1e-3;
  Scalar init_gyro_bias_var = 1e-6;
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

  const EkfState& step(double stamp, const ImuMeasurement& imu);
  void update_pose(const PoseType& pose, double stamp);

  // Omega and b are the pre-weighted information matrix/vector.
  // Set update_cov=true only on the final IEKF iteration.
  void update_points(const Eigen::Matrix<Scalar, 15, 15>& Omega,
                     const Eigen::Matrix<Scalar, 15, 1>& b,
                     bool update_cov = true);
  PoseType pose() const;

  EkfState state;
  EkfConfig& config;
  EkfStatus status = EkfStatus::Calibrating;
  bool is_stationary = false;
  double last_stamp = -1.0;

 private:
  void update_stillness(const ImuMeasurement& imu, double stamp);
  void predict(const ImuMeasurement& imu, double stamp);
  void init_from_calib();
  void zupt();

  std::vector<Vec3> calib_accels;
  std::vector<Vec3> calib_gyros;
  std::deque<std::pair<double, ImuMeasurement>> still_buf;

  Vec3 still_sum_a = Vec3::Zero();
  Vec3 still_sum_g = Vec3::Zero();
  Scalar still_sumsq_a = 0.0;
  Scalar still_sumsq_g = 0.0;
};

PoseType interp_pose(const std::vector<std::pair<double, PoseType>>& traj,
                     double t);

// Motion-compensates points already in IMU frame to the scan-end IMU frame (traj.back()).
void deskew_cloud(PCloudType& cloud, double stamp,
                  const std::vector<std::pair<double, PoseType>>& traj);

}  // namespace fodom
