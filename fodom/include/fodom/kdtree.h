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
  Scalar leaf_extent_sq = 0.04;

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

  const Vec3& nn_fast(const Vec3& point) const;
  const Vec3& nn_exact(const Vec3& point) const;

  const TreeNode& nn_plane(const Vec3& point) const;
};

void get_leaves(const TreeNode& tree, std::vector<Vec3>& leaves);

// Applies a rigid transform to every point in the cloud in-place.
void transform_cloud(PCloudType& cloud, const PoseType& T);

}  // namespace fodom
