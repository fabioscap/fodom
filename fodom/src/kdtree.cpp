#include "fodom/kdtree.h"

#include <thread>

namespace fodom {

TreeNode::TreeNode(PCloudType::iterator begin, PCloudType::iterator end,
                   const KdTreeConfig& config, size_t level) {
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

  const bool do_split = n_left > 0 && n_right > 0 &&
                        (n_left > config.max_points_per_leaf ||
                         n_right > config.max_points_per_leaf) &&
                        solver.eigenvalues()(2) > config.leaf_extent_sq &&
                        next_level < config.max_depth;
  if (!do_split) return;

  if (level < config.max_parallel_level) {
    auto build_left = [this, begin, mid, &config, next_level]() {
      left = std::make_unique<TreeNode>(begin, mid, config, next_level);
    };
    std::thread leftthread(build_left);
    right = std::make_unique<TreeNode>(mid, end, config, next_level);
    leftthread.join();
  } else {
    left = std::make_unique<TreeNode>(begin, mid, config, next_level);
    right = std::make_unique<TreeNode>(mid, end, config, next_level);
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
  if (!left && !right) return *this;
  if (!left) return right->nn_plane(point);
  if (!right) return left->nn_plane(point);

  if ((point - mean).dot(split_plane) < 0)
    return left->nn_plane(point);
  else
    return right->nn_plane(point);
}

void get_leaves(const TreeNode& tree, std::vector<Vec3>& leaves) {
  if (!tree.left && !tree.right)
    leaves.push_back(tree.mean);
  else {
    get_leaves(*tree.left, leaves);
    get_leaves(*tree.right, leaves);
  }
}

void transform_cloud(PCloudType& cloud, const PoseType& T) {
  for (auto& p : cloud) p.xyz = T * p.xyz;
}

}  // namespace fodom
