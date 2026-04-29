#include <fodom/kdtree.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>

static fodom::PCloudType load_cloud(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("Cannot open file: " + path);

  uint64_t n = 0;
  f.read(reinterpret_cast<char*>(&n), sizeof(n));

  fodom::PCloudType cloud(n);
  f.read(reinterpret_cast<char*>(cloud.data()), n * 3 * sizeof(fodom::Scalar));

  if (!f)
    throw std::runtime_error("Failed to read " + std::to_string(n) + " points");

  return cloud;
}

static void get_leaves(const fodom::TreeNode& node,
                       std::vector<const fodom::TreeNode*>& leaves) {
  if (!node.left && !node.right) {
    leaves.push_back(&node);
  } else {
    if (node.left) get_leaves(*node.left, leaves);
    if (node.right) get_leaves(*node.right, leaves);
  }
}

static void print_pose(const std::string& label, const fodom::PoseType& pose) {
  std::cout << label << ":\n";
  std::cout << "  R =\n" << pose.rotation() << "\n";
  std::cout << "  t = " << pose.translation().transpose() << "\n\n";
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: benchmark_match <cloud.bin>\n";
    return 1;
  }

  auto cloud = load_cloud(argv[1]);
  cloud.erase(
      std::remove_if(cloud.begin(), cloud.end(),
                     [](const fodom::Vec3& p) { return !p.allFinite(); }),
      cloud.end());
  std::cout << "Loaded " << cloud.size() << " points\n";

  // known perturbation: small rotation (5 deg around Z) + translation
  fodom::PoseType perturbation = fodom::PoseType::Identity();
  double angle = 5.0 * M_PI / 180.0;
  perturbation.linear() =
      Eigen::AngleAxisd(angle, fodom::Vec3::UnitZ()).toRotationMatrix();
  perturbation.translation() = fodom::Vec3(0.1, -0.05, 0.02);

  std::cout << "Applied perturbation:\n";
  print_pose("  perturbation", perturbation);

  // create corrupted moving cloud: moving = perturbation * fixed
  fodom::PCloudType moving_cloud(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    moving_cloud[i] = perturbation * cloud[i];
  }

  // build trees
  fodom::KdTreeConfig config;
  // config.max_depth = 300;
  // config.max_points_per_leaf = 4;
  auto t0 = std::chrono::high_resolution_clock::now();
  auto fixed_tree =
      std::make_unique<fodom::TreeNode>(cloud.begin(), cloud.end(), config);
  auto t1 = std::chrono::high_resolution_clock::now();
  auto moving_tree = std::make_unique<fodom::TreeNode>(
      moving_cloud.begin(), moving_cloud.end(), config);
  auto t2 = std::chrono::high_resolution_clock::now();

  std::cout << "Fixed tree built in "
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms\n";
  std::cout << "Moving tree built in "
            << std::chrono::duration<double, std::milli>(t2 - t1).count()
            << " ms\n\n";

  // test: each leaf's mean should match back to itself
  {
    std::vector<const fodom::TreeNode*> leaves;
    get_leaves(*fixed_tree, leaves);
    size_t passed = 0;
    size_t failed = 0;
    double max_dist = 0;
    for (const auto* leaf : leaves) {
      const auto& matched = fixed_tree->nn_fast(leaf->mean);
      if (&matched == leaf) {
        ++passed;
      } else {
        ++failed;
        double dist = (matched.mean - leaf->mean).norm();
        max_dist = std::max(max_dist, dist);
      }
    }
    std::cout << "Self-match test: " << passed << " passed, " << failed
              << " failed out of " << leaves.size() << " leaves\n";
    if (failed > 0)
      std::cout << "  max distance between expected and matched leaf: "
                << max_dist << "\n";
    std::cout << "\n";
  }

  // initial guess = the perturbation itself (perfect initialization)
  // fodom::PoseType initial_guess = perturbation;
  fodom::PoseType initial_guess = fodom::PoseType::Identity();

  // match_point_cloud: should recover identity (T such that T * moving ≈ fixed)
  auto t3 = std::chrono::high_resolution_clock::now();
  auto pose_pc =
      fodom::match_point_cloud(*fixed_tree, moving_cloud, initial_guess).pose;
  auto t4 = std::chrono::high_resolution_clock::now();
  std::cout << "match_point_cloud: "
            << std::chrono::duration<double, std::milli>(t4 - t3).count()
            << " ms\n";
  print_pose("match_point_cloud result", pose_pc);

  // check how close results are to identity
  auto check = [](const std::string& name, const fodom::PoseType& pose) {
    double rot_err = Eigen::AngleAxisd(pose.rotation()).angle() * 180.0 / M_PI;
    double trans_err = pose.translation().norm();
    std::cout << name << " error: rot=" << rot_err
              << " deg, trans=" << trans_err << " m\n";
  };
  check("match_point_cloud", pose_pc);

  return 0;
}
