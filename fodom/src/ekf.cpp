#include "fodom/ekf.h"

#include <algorithm>
#include <iostream>

namespace fodom {

using Mat6 = Eigen::Matrix<Scalar, 6, 6>;
using Mat15 = Eigen::Matrix<Scalar, 15, 15>;
using Vec6 = Eigen::Matrix<Scalar, 6, 1>;
using Vec15 = Eigen::Matrix<Scalar, 15, 1>;
using Mat3x15 = Eigen::Matrix<Scalar, 3, 15>;
using Mat6x15 = Eigen::Matrix<Scalar, 6, 15>;
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
  state.rotation = Eigen::Quaternion<Scalar>::FromTwoVectors(
                       mean_accel.normalized(), (-config.gravity).normalized())
                       .toRotationMatrix();
  state.accel_bias =
      mean_accel - state.rotation.transpose() * (-config.gravity);

  state.P.setZero();
  state.P.block<3, 3>(0, 0).diagonal().setConstant(config.init_pos_var);
  state.P.block<3, 3>(3, 3).diagonal().setConstant(config.init_vel_var);
  state.P.block<3, 3>(6, 6).diagonal().setConstant(config.init_rot_var);
  state.P.block<3, 3>(9, 9).diagonal().setConstant(config.init_accel_bias_var);
  state.P.block<3, 3>(12, 12).diagonal().setConstant(config.init_gyro_bias_var);

  status = EkfStatus::Running;
}

void Ekf::update_stillness(const ImuMeasurement& imu, double stamp) {
  still_buf.push_back({stamp, imu});
  still_sum_a += imu.accel;
  still_sum_g += imu.gyro;
  still_sumsq_a += imu.accel.squaredNorm();
  still_sumsq_g += imu.gyro.squaredNorm();

  while (stamp - still_buf.front().first > config.still_window) {
    const auto& old = still_buf.front().second;
    still_sum_a -= old.accel;
    still_sum_g -= old.gyro;
    still_sumsq_a -= old.accel.squaredNorm();
    still_sumsq_g -= old.gyro.squaredNorm();
    still_buf.pop_front();
  }

  const double window_elapsed = stamp - still_buf.front().first;
  if (window_elapsed >= config.still_window) {
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

const EkfState& Ekf::step(double stamp, const ImuMeasurement& imu) {
  update_stillness(imu, stamp);

  if (status == EkfStatus::Calibrating) {
    if (calib_accels.empty())
      std::cerr << "[fodom] calibrating — keep sensor still\n";

    calib_accels.push_back(imu.accel);
    calib_gyros.push_back(imu.gyro);

    if (calib_accels.size() >= config.calib_samples) {
      init_from_calib();
      calib_accels.clear();
      calib_gyros.clear();
      last_stamp = stamp;
      std::cerr << "[fodom] calibration done\n";
    }
  } else if (status == EkfStatus::Running) {
    predict(imu, stamp);
    if (config.do_zupt and is_stationary) zupt();
  }

  return state;
}

void Ekf::predict(const ImuMeasurement& imu, double stamp) {
  if (last_stamp < 0.0) {
    last_stamp = stamp;
    return;
  }

  const Scalar dt = static_cast<Scalar>(stamp - last_stamp);
  last_stamp = stamp;

  const Vec3 a = imu.accel - state.accel_bias;
  const Vec3 w = imu.gyro - state.gyro_bias;

  const Mat3 R0 = state.rotation;
  const Vec3 a_world = R0 * a + config.gravity;

  state.position += state.velocity * dt + 0.5 * a_world * dt * dt;
  state.velocity += a_world * dt;
  state.rotation = normalize_so3(R0 * exp_so3(w * dt));

  // First-order covariance propagation
  const Mat3 B = -dt * R0 * skew(a);
  const Mat3 C = -dt * R0;
  const Mat3 Dw = -dt * skew(w);

  const Eigen::Matrix<Scalar, 3, 15> dFP_pos = dt * state.P.block<3, 15>(3, 0);
  const Eigen::Matrix<Scalar, 3, 15> dFP_vel =
      B * state.P.block<3, 15>(6, 0) + C * state.P.block<3, 15>(9, 0);
  const Eigen::Matrix<Scalar, 3, 15> dFP_rot =
      Dw * state.P.block<3, 15>(6, 0) - dt * state.P.block<3, 15>(12, 0);

  state.P.block<3, 15>(0, 0) += dFP_pos;
  state.P.block<15, 3>(0, 0) += dFP_pos.transpose();
  state.P.block<3, 15>(3, 0) += dFP_vel;
  state.P.block<15, 3>(0, 3) += dFP_vel.transpose();
  state.P.block<3, 15>(6, 0) += dFP_rot;
  state.P.block<15, 3>(0, 6) += dFP_rot.transpose();

  state.P.block<3, 3>(3, 3).diagonal().array() +=
      config.accel_noise * config.accel_noise * dt;
  state.P.block<3, 3>(6, 6).diagonal().array() +=
      config.gyro_noise * config.gyro_noise * dt;
  state.P.block<3, 3>(9, 9).diagonal().array() +=
      config.accel_bias_rw * config.accel_bias_rw * dt;
  state.P.block<3, 3>(12, 12).diagonal().array() +=
      config.gyro_bias_rw * config.gyro_bias_rw * dt;
}

template <int N>
static void ekf_update_(EkfState& state, const Eigen::Matrix<Scalar, N, 15>& H,
                        const Eigen::Matrix<Scalar, N, 1>& z,
                        const Eigen::Matrix<Scalar, N, N>& R,
                        bool update_P = true) {
  using Mat15xN = Eigen::Matrix<Scalar, 15, N>;
  using MatNN = Eigen::Matrix<Scalar, N, N>;

  const MatNN S = H * state.P * H.transpose() + R;
  const Mat15xN K = state.P * H.transpose() * S.inverse();
  const Vec15 dx = K * z;

  state.position += dx.template segment<3>(0);
  state.velocity += dx.template segment<3>(3);
  state.rotation =
      normalize_so3(state.rotation * exp_so3(dx.template segment<3>(6)));
  state.accel_bias += dx.template segment<3>(9);
  state.gyro_bias += dx.template segment<3>(12);

  if (update_P) {
    const Mat15 I_KH = Mat15::Identity() - K * H;
    state.P = I_KH * state.P * I_KH.transpose() + K * R * K.transpose();
  }
}

template <int N>
static void ekf_update(EkfState& state, const Eigen::Matrix<Scalar, N, 15>& H,
                       const Eigen::Matrix<Scalar, N, 1>& z,
                       const Eigen::Matrix<Scalar, N, N>& R,
                       size_t n_iters = 1) {
  for (size_t i = 0; i < n_iters - 1; ++i) ekf_update_(state, H, z, R, false);
  ekf_update_(state, H, z, R, true);
}

template <int N>
static void ekf_update(EkfState& state, const Eigen::Matrix<Scalar, N, 15>& H,
                       const Eigen::Matrix<Scalar, N, 1>& z, Scalar sigma,
                       size_t n_iters = 1) {
  const Eigen::Matrix<Scalar, N, N> R =
      (sigma * sigma) * Eigen::Matrix<Scalar, N, N>::Identity();
  ekf_update(state, H, z, R, n_iters);
}

void Ekf::update_pose(const PoseType& pose, double stamp) {
  if (status != EkfStatus::Running) return;

  Mat6x15 H = Mat6x15::Zero();
  H.block<3, 3>(0, 0) = Mat3::Identity();
  H.block<3, 3>(3, 6) = Mat3::Identity();

  Vec6 z;
  z.head<3>() = pose.translation() - state.position;
  z.tail<3>() = log_so3(state.rotation.transpose() * pose.linear());

  Eigen::Matrix<Scalar, 6, 6> R = Eigen::Matrix<Scalar, 6, 6>::Zero();
  R.diagonal().head<3>().setConstant(config.pos_noise * config.pos_noise);
  R.diagonal().tail<3>().setConstant(config.rot_noise * config.rot_noise);

  ekf_update<6>(state, H, z, R);
  last_stamp = stamp;
}

void Ekf::update_points(const Mat15& Omega, const Vec15& b, bool update_cov) {
  if (status != EkfStatus::Running) return;

  const Eigen::LDLT<Mat15> ldlt(state.P.inverse() + Omega);
  const Vec15 dx = ldlt.solve(b);

  state.position += dx.segment<3>(0);
  state.velocity += dx.segment<3>(3);
  state.rotation = normalize_so3(state.rotation * exp_so3(dx.segment<3>(6)));
  state.accel_bias += dx.segment<3>(9);
  state.gyro_bias += dx.segment<3>(12);
  if (update_cov) state.P = ldlt.solve(Mat15::Identity());
}

void Ekf::zupt() {
  if (status != EkfStatus::Running) return;

  Mat3x15 H = Mat3x15::Zero();
  H.block<3, 3>(0, 3) = Mat3::Identity();

  const Vec3 z = -state.velocity;
  ekf_update<3>(state, H, z, config.zupt_noise);
}

PoseType Ekf::pose() const {
  PoseType p = PoseType::Identity();
  p.linear() = state.rotation;
  p.translation() = state.position;
  return p;
}

PoseType interp_pose(const std::vector<std::pair<double, PoseType>>& traj,
                     double t) {
  if (t <= traj.front().first) return traj.front().second;
  if (t >= traj.back().first) return traj.back().second;
  auto it =
      std::lower_bound(traj.begin(), traj.end(), t,
                       [](const auto& e, double v) { return e.first < v; });
  const auto& [t1, P1] = *it;
  const auto& [t0, P0] = *std::prev(it);
  const double a = (t - t0) / (t1 - t0);
  PoseType out;
  out.translation() =
      P0.translation() + a * (P1.translation() - P0.translation());
  out.linear() = Eigen::Quaterniond(P0.rotation())
                     .slerp(a, Eigen::Quaterniond(P1.rotation()))
                     .toRotationMatrix();
  return out;
}

void deskew_cloud(PCloudType& cloud, double stamp,
                  const std::vector<std::pair<double, PoseType>>& traj) {
  const PoseType T_W_I_ref_inv = traj.back().second.inverse();
  for (auto& p : cloud) {
    const double t_abs = stamp + static_cast<double>(p.nsec) * 1e-9;
    p.xyz = (T_W_I_ref_inv * interp_pose(traj, t_abs)) * p.xyz;
  }
}

}  // namespace fodom
