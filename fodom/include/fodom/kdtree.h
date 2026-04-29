#pragma once

#include <memory>

#include "fodom/math.h"

namespace fodom {

struct PointType {
  Vec3 xyz;
  size_t nsec{0};
};

using PCloudType = std::vector<PointType>;

struct KdTreeConfig {
  size_t max_points_per_leaf = 20;
  size_t max_depth = 16;
  size_t max_parallel_level = 3;
};

struct TreeNode {
  Vec3 mean;
  Vec3 normal;
  Vec3 split_plane;

  size_t num_points;

  std::unique_ptr<TreeNode> left;
  std::unique_ptr<TreeNode> right;

  TreeNode(PCloudType::iterator begin, PCloudType::iterator end,
           const KdTreeConfig& config = KdTreeConfig(), size_t level = 0);

  const TreeNode& nn_fast(const Vec3& point) const;
  const TreeNode& nn_exact(const Vec3& point) const;
};

struct ICPConfig {
  size_t n_iters = 100;
  Scalar damping = 10;
  Scalar max_correspondence_dist = 1.0;
};

struct ICPResult {
  PoseType pose;
  Scalar ratio{0.0};
};

void get_leaves(const TreeNode& tree, std::vector<Vec3>& leaves);

ICPResult match_point_cloud(
    const TreeNode& fixed, const PCloudType& moving,
    const PoseType& initial_guess = PoseType::Identity(),
    const ICPConfig& config = ICPConfig());

}  // namespace fodom
