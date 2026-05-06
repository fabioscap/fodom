#include "fodom/kdtree.h"

#include <thread>

namespace fodom {

TreeNode::TreeNode(PCloudType::iterator begin, PCloudType::iterator end,
                   const KdTreeConfig& config, size_t level,
                   const TreeNode* ppl) {
  const size_t n = std::distance(begin, end);
  num_points = n;

  Vec3 m = Vec3::Zero();
  Mat3 cov = Mat3::Zero();
  for (auto it = begin; it != end; ++it) {
    const Vec3& v = (*it).xyz;
    m += v;
    cov += v * v.transpose();
  }
  m *= (1. / n);
  cov *= (1. / n);
  cov -= m * m.transpose();

  mean = m;

  Eigen::SelfAdjointEigenSolver<Mat3> solver;
  solver.computeDirect(cov);
  normal = solver.eigenvectors().col(0);
  split_plane = solver.eigenvectors().col(2);

  auto mid = std::partition(begin, end, [&](const PointType& p) {
    return (p.xyz - mean).dot(split_plane) < 0.0;
  });

  const size_t n_left = std::distance(begin, mid);
  const size_t n_right = std::distance(mid, end);
  const size_t next_level = level + 1;

  planarity = solver.eigenvalues()(0) / (solver.eigenvalues()(2) + 1e-10);
  parent_plane = (planarity < config.planar_ratio) ? this : ppl;

  const bool do_split = n_left > 0 && n_right > 0 &&
                        (n_left > config.max_points_per_leaf ||
                         n_right > config.max_points_per_leaf) &&
                        solver.eigenvalues()(2) > config.leaf_extent_sq &&
                        next_level < config.max_depth;
  if (!do_split) return;

  if (level < config.max_parallel_level) {
    auto build_left = [this, begin, mid, &config, next_level]() {
      left = std::make_unique<TreeNode>(begin, mid, config, next_level,
                                        parent_plane);
    };
    std::thread leftthread(build_left);
    right =
        std::make_unique<TreeNode>(mid, end, config, next_level, parent_plane);
    leftthread.join();
  } else {
    left = std::make_unique<TreeNode>(begin, mid, config, next_level,
                                      parent_plane);
    right =
        std::make_unique<TreeNode>(mid, end, config, next_level, parent_plane);
  }
}

const Vec3& TreeNode::nn_fast(const Vec3& point) const {
  if (!left && !right) return mean;
  if (!left) return right->mean;
  if (!right) return left->mean;

  if ((point - mean).dot(split_plane) < 0)
    return left->nn_fast(point);
  else
    return right->nn_fast(point);
}

const Vec3& TreeNode::nn_exact(const Vec3& point) const {
  if (!left && !right) return mean;
  if (!left) return right->nn_exact(point);
  if (!right) return left->nn_exact(point);

  const auto& best_left = left->nn_exact(point);
  const auto& best_right = right->nn_exact(point);

  Scalar d_left = (point - best_left).squaredNorm();
  Scalar d_right = (point - best_right).squaredNorm();

  return d_left < d_right ? best_left : best_right;
}

const TreeNode& TreeNode::nn_plane(const Vec3& point) const {
  if (!left && !right)
    return (parent_plane == this || !parent_plane) ? *this : *parent_plane;
  if (!left) return right->nn_plane(point);
  if (!right) return left->nn_plane(point);

  if ((point - mean).dot(split_plane) < 0)
    return left->nn_plane(point);
  else
    return right->nn_plane(point);
}

ICPResult match_kdtree(const TreeNode& fixed, const std::vector<Vec3>& moving,
                       const PoseType& initial_guess, const ICPConfig& config) {
  PoseType X{initial_guess};

  Eigen::Matrix<Scalar, 6, 6> H;
  Eigen::Vector<Scalar, 6> b;

  const Scalar max_dist_sq =
      config.max_correspondence_dist * config.max_correspondence_dist;
  size_t n_correspondences{0};
  for (size_t i{0}; i < config.n_iters; ++i) {
    n_correspondences = 0;
    H.setZero();
    b.setZero();

    for (const auto& point : moving) {
      const Vec3 point_transformed = X * point;
      const auto& best_leaf = fixed.nn_plane(point_transformed);

      if ((point_transformed - best_leaf.mean).squaredNorm() > max_dist_sq)
        continue;

      n_correspondences++;

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

  ICPResult result;
  result.pose = X;
  result.ratio = static_cast<Scalar>(n_correspondences) /
                 static_cast<Scalar>(moving.size());
  return result;
}

void get_leaves(const TreeNode& tree, std::vector<Vec3>& leaves) {
  if (!tree.left && !tree.right)
    leaves.push_back(tree.mean);
  else {
    get_leaves(*tree.left, leaves);
    get_leaves(*tree.right, leaves);
  }
}

}  // namespace fodom
