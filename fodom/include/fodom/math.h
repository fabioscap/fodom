#pragma once

#include <Eigen/Dense>
#include <vector>

namespace fodom {

using Scalar = double;
using Vec3 = Eigen::Vector3<Scalar>;
using Mat3 = Eigen::Matrix3<Scalar>;
using PoseType = Eigen::Transform<double, 3, Eigen::Isometry>;

constexpr Scalar small_value = 1e-6;

inline Mat3 skew(const Vec3& w) {
  Mat3 S;
  S << 0.0, -w.z(), w.y(), w.z(), 0.0, -w.x(), -w.y(), w.x(), 0.0;
  return S;
}

// skew(u)^2 == u*u^T - ||u||^2 * I  avoids the 3x3 matrix multiply
inline Mat3 exp_so3(const Vec3& w) {
  const Scalar theta = w.norm();
  if (theta < small_value) return Mat3::Identity();
  const Vec3 axis = w / theta;
  const Scalar s = std::sin(theta), c = std::cos(theta);
  return c * Mat3::Identity() + s * skew(axis) +
         (1.0 - c) * axis * axis.transpose();
}

inline Mat3 exp_cayley(const Vec3& w) {
  const Vec3 v = w / 2;
  const Scalar n2 = v.dot(v);
  const Scalar f = 2.0 / (1.0 + n2);
  return ((1.0 - n2) / (1.0 + n2)) * Mat3::Identity() + f * skew(v) +
         f * v * v.transpose();
}

inline Mat3 normalize_so3(const Mat3& R) {
  Eigen::Quaterniond q(R);
  q.normalize();
  return q.toRotationMatrix();
}

inline Vec3 log_so3(const Mat3& R) {
  const Scalar theta = std::acos((R.trace() - 1.0) * 0.5);
  if (theta < small_value) return Vec3::Zero();
  return (theta / (2.0 * std::sin(theta))) *
         Vec3(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
}

}  // namespace fodom
