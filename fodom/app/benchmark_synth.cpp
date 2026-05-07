#include <polyscope/point_cloud.h>
#include <polyscope/polyscope.h>

#include <chrono>
#include <iostream>
#include <random>
#include <vector>

#include "fodom/icp.h"

static fodom::PCloudType generate_three_planes(size_t n_per_plane,
                                               double noise) {
  // Plane 1: ground (Z=0), normal along Z
  // Plane 2: wall   (Y=0), normal along Y
  // Plane 3: wall   (X=5), normal along X
  // Three independent normals fully constrain all 6 DOF of SE(3).

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> span(-5.0, 5.0);
  std::uniform_real_distribution<double> span_pos(0.0, 5.0);
  std::normal_distribution<double> noise_dist(0.0, noise);

  fodom::PCloudType cloud;
  cloud.reserve(3 * n_per_plane);

  // Ground plane: z ≈ 0
  for (size_t i = 0; i < n_per_plane; ++i) {
    cloud.emplace_back(span(rng), span(rng), noise_dist(rng));
  }

  // Wall 1: y ≈ 0
  for (size_t i = 0; i < n_per_plane; ++i) {
    cloud.emplace_back(span(rng), noise_dist(rng), span_pos(rng));
  }

  // Wall 2: x ≈ 5
  for (size_t i = 0; i < n_per_plane; ++i) {
    cloud.emplace_back(5.0 + noise_dist(rng), span(rng), span_pos(rng));
  }

  return cloud;
}

static fodom::PCloudType transform_cloud(const fodom::PCloudType& cloud,
                                         const fodom::PoseType& T) {
  fodom::PCloudType out(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) out[i] = T * cloud[i];
  return out;
}

struct State {
  polyscope::PointCloud* pc_fixed = nullptr;
  polyscope::PointCloud* pc_initial = nullptr;
  polyscope::PointCloud* pc_result = nullptr;
  float point_radius = 0.05f;
  bool show_fixed = true;
  bool show_initial = true;
  bool show_result = true;
};

static State g_state;

static void callback() {
  if (ImGui::IsKeyPressed(ImGuiKey_F, false)) {
    g_state.show_fixed = !g_state.show_fixed;
    g_state.pc_fixed->setEnabled(g_state.show_fixed);
    std::cout << "Fixed: " << (g_state.show_fixed ? "on" : "off") << "\n";
  }
  if (ImGui::IsKeyPressed(ImGuiKey_I, false)) {
    g_state.show_initial = !g_state.show_initial;
    g_state.pc_initial->setEnabled(g_state.show_initial);
    std::cout << "Initial guess: " << (g_state.show_initial ? "on" : "off")
              << "\n";
  }
  if (ImGui::IsKeyPressed(ImGuiKey_R, false)) {
    g_state.show_result = !g_state.show_result;
    g_state.pc_result->setEnabled(g_state.show_result);
    std::cout << "Result: " << (g_state.show_result ? "on" : "off") << "\n";
  }

  if (ImGui::IsKeyPressed(ImGuiKey_Equal, true) ||
      ImGui::IsKeyPressed(ImGuiKey_KeypadAdd, true)) {
    g_state.point_radius = std::min(g_state.point_radius * 1.25f, 0.5f);
    g_state.pc_fixed->setPointRadius(g_state.point_radius);
    g_state.pc_initial->setPointRadius(g_state.point_radius);
    g_state.pc_result->setPointRadius(g_state.point_radius);
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Minus, true) ||
      ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract, true)) {
    g_state.point_radius = std::max(g_state.point_radius / 1.25f, 0.001f);
    g_state.pc_fixed->setPointRadius(g_state.point_radius);
    g_state.pc_initial->setPointRadius(g_state.point_radius);
    g_state.pc_result->setPointRadius(g_state.point_radius);
  }
}

int main() {
  const size_t n_per_plane = 500;
  const double noise = 0.00;

  auto cloud = generate_three_planes(n_per_plane, noise);
  std::cout << "Generated " << cloud.size() << " points (three planes)\n";

  // Ground-truth perturbation: 10 deg around Z + translation
  fodom::PoseType perturbation = fodom::PoseType::Identity();
  double angle = 10 * M_PI / 180.0;
  perturbation.linear() =
      Eigen::AngleAxisd(angle, fodom::Vec3::UnitZ()).toRotationMatrix();
  perturbation.translation() = fodom::Vec3(1, 5., 0);

  auto moving_cloud(cloud);

  // Build fixed tree
  fodom::KdTreeConfig kd_config;
  kd_config.max_parallel_level = 0;
  kd_config.max_points_per_leaf = 3;
  fodom::PCloudType fixed_cloud = cloud;  // copy, constructor reorders
  auto fixed_tree = std::make_unique<fodom::TreeNode>(
      fixed_cloud.begin(), fixed_cloud.end(), kd_config);

  auto t0 = std::chrono::high_resolution_clock::now();

  fodom::IcpConfig icp_config;
  icp_config.n_iters = 10;
  icp_config.damping = 0;
  auto result_icp = fodom::match_point_cloud(*fixed_tree, moving_cloud,
                                             perturbation, icp_config);
  auto result = result_icp.pose;
  auto t1 = std::chrono::high_resolution_clock::now();

  std::cout << "ICP took "
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms\n";

  auto initial_cloud = transform_cloud(moving_cloud, perturbation);
  auto result_cloud = transform_cloud(moving_cloud, result);

  // Visualize
  polyscope::options::buildGui = false;
  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
  polyscope::init();

  g_state.pc_fixed =
      polyscope::registerPointCloud("fixed (ground truth)", fixed_cloud);
  g_state.pc_fixed->setPointRadius(g_state.point_radius);
  g_state.pc_fixed->setPointColor({0.2, 0.8, 0.2});

  g_state.pc_initial =
      polyscope::registerPointCloud("initial guess", initial_cloud);
  g_state.pc_initial->setPointRadius(g_state.point_radius);
  g_state.pc_initial->setPointColor({0.8, 0.2, 0.2});

  g_state.pc_result = polyscope::registerPointCloud("ICP result", result_cloud);
  g_state.pc_result->setPointRadius(g_state.point_radius);
  g_state.pc_result->setPointColor({0.2, 0.4, 0.9});

  std::cout << "Controls: F=toggle fixed, I=toggle initial, R=toggle result, "
               "+/-=point size\n";

  polyscope::state::userCallback = callback;
  polyscope::show();

  return 0;
}
