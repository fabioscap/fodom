#include "fodom/icp.h"

#include <cmath>

namespace fodom {

IcpContrib accumulate_keyframe(
    const TreeNode& fixed, const PoseType& fixed_pose,
    const std::vector<Vec3>& moving, const PoseType& moving_pose,
    Scalar inv_var,      // inverse variance of point corrections
    Scalar max_dist_sq,  // hard cut-off
    Scalar huber_delta) {
  const PoseType moving_in_fixed = fixed_pose.inverse() * moving_pose;
  const Mat3& imu_rot = moving_pose.linear();
  const Vec3& imu_pos = moving_pose.translation();

  IcpContrib result;
  for (int i = 0; i < static_cast<int>(moving.size()); ++i) {
    const Vec3& point = moving[i];
    const Vec3 point_transformed = moving_in_fixed * point;
    const TreeNode& matched = fixed.nn_plane(point_transformed);
    if ((point_transformed - matched.mean).squaredNorm() > max_dist_sq)
      continue;

    const Vec3 n_world = fixed_pose.linear() * matched.normal;
    const Vec3 mu_world = fixed_pose * matched.mean;
    const Vec3 p_world = imu_rot * point + imu_pos;

    Eigen::Matrix<Scalar, 1, 15> J;
    J.block<1, 3>(0, 0) = n_world.transpose();
    J.block<1, 3>(0, 3).setZero();
    J.block<1, 3>(0, 6) = -n_world.transpose() * imu_rot * skew(point);
    J.block<1, 6>(0, 9).setZero();
    const Scalar z = n_world.dot(mu_world - p_world);

    const Scalar cos_inc =
        std::abs(n_world.dot((imu_rot * point).normalized()));
    const Scalar range_sq = point.squaredNorm();
    const Scalar abs_z = std::abs(z);
    const Scalar w_huber =
        (abs_z <= huber_delta) ? Scalar(1) : huber_delta / abs_z;
    const Scalar w = inv_var * cos_inc / (range_sq + 1e-6) * w_huber;

    result.H += w * J.transpose() * J;
    result.b += w * J.transpose() * z;
    ++result.valid;
  }
  return result;
}

MatchResult match_kdtree(const TreeNode& fixed, const std::vector<Vec3>& moving,
                         const PoseType& initial_guess,
                         const IcpConfig& config) {
  PoseType X{initial_guess};

  Eigen::Matrix<Scalar, 6, 6> H;
  Eigen::Matrix<Scalar, 6, 1> b;

  const Scalar max_dist_sq =
      config.max_correspondence_dist * config.max_correspondence_dist;
  size_t n_correspondences = 0;

  for (size_t i = 0; i < config.n_iters; ++i) {
    n_correspondences = 0;
    H.setZero();
    b.setZero();

    for (const auto& point : moving) {
      const Vec3 point_transformed = X * point;
      const TreeNode& best_leaf = fixed.nn_plane(point_transformed);

      if ((point_transformed - best_leaf.mean).squaredNorm() > max_dist_sq)
        continue;

      ++n_correspondences;

      const Scalar error =
          (point_transformed - best_leaf.mean).dot(best_leaf.normal);

      Eigen::Matrix<Scalar, 1, 6> J;
      J << best_leaf.normal.transpose(),
          -best_leaf.normal.transpose() * X.linear() * skew(point);

      b += J.transpose() * error;
      H += J.transpose() * J;
    }

    const Eigen::Matrix<Scalar, 6, 1> dx =
        -(H + config.damping * Eigen::Matrix<Scalar, 6, 6>::Identity())
             .ldlt()
             .solve(b);

    X.translation() += dx.head<3>();
    X.linear() = X.linear() * exp_cayley(dx.tail<3>());
  }

  MatchResult result;
  result.pose = X;
  result.ratio = static_cast<Scalar>(n_correspondences) /
                 static_cast<Scalar>(moving.size());
  return result;
}

}  // namespace fodom
