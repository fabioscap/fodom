#pragma once

#include "fodom/ekf.h"
#include "fodom/kdtree.h"

namespace fodom {

struct IcpConfig {
  size_t n_iters = 10;
  Scalar damping = 0.0;  // for standalone match_kdtree
  Scalar max_correspondence_dist = 1.0;
  Scalar huber_delta = 0.1;  // [m] Huber threshold on point-to-plane residual
};

struct IcpContrib {
  Eigen::Matrix<Scalar, 15, 15> H = Eigen::Matrix<Scalar, 15, 15>::Zero();
  Eigen::Matrix<Scalar, 15, 1> b = Eigen::Matrix<Scalar, 15, 1>::Zero();
  int valid = 0;
};

struct MatchResult {
  PoseType pose;
  Scalar ratio{0.0};
};

// IEKF-coupled: accumulates (H, b) information contribution from one keyframe.
// leaves and kf tree points are both in their respective IMU frames.
IcpContrib accumulate_keyframe(
    const TreeNode& fixed, const PoseType& fixed_pose,
    const std::vector<Vec3>& moving, const PoseType& moving_pose,
    Scalar inv_var,      // inverse variance of point corrections
    Scalar max_dist_sq,  // hard cut-off
    Scalar huber_delta);

// Standalone cloud-to-cloud ICP, returns moving-in-fixed pose
MatchResult match_kdtree(const TreeNode& fixed, const std::vector<Vec3>& moving,
                         const PoseType& initial_guess = PoseType::Identity(),
                         const IcpConfig& config = IcpConfig());

}  // namespace fodom
