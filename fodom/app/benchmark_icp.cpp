#include <polyscope/point_cloud.h>
#include <polyscope/polyscope.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "fodom/kdtree.h"

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
  float point_radius = 0.002f;
  bool show_fixed = true;
  bool show_initial = true;
  bool show_result = true;
};

static State g_state;

static void callback() {
  // toggle visibility: F=fixed, I=initial, R=result
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

  // point size
  if (ImGui::IsKeyPressed(ImGuiKey_Equal, true) ||
      ImGui::IsKeyPressed(ImGuiKey_KeypadAdd, true)) {
    g_state.point_radius = std::min(g_state.point_radius * 1.25f, 0.1f);
    g_state.pc_fixed->setPointRadius(g_state.point_radius);
    g_state.pc_initial->setPointRadius(g_state.point_radius);
    g_state.pc_result->setPointRadius(g_state.point_radius);
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Minus, true) ||
      ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract, true)) {
    g_state.point_radius = std::max(g_state.point_radius / 1.25f, 0.0001f);
    g_state.pc_fixed->setPointRadius(g_state.point_radius);
    g_state.pc_initial->setPointRadius(g_state.point_radius);
    g_state.pc_result->setPointRadius(g_state.point_radius);
  }
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: benchmark_icp <cloud.bin>\n";
    return 1;
  }

  auto cloud = load_cloud(argv[1]);
  cloud.erase(
      std::remove_if(cloud.begin(), cloud.end(),
                     [](const fodom::Vec3& p) { return !p.allFinite(); }),
      cloud.end());
  std::cout << "Loaded " << cloud.size() << " points\n";

  // perturbation: 15 deg around Z
  fodom::PoseType initial_guess = fodom::PoseType::Identity();
  double angle = 15 * M_PI / 180.0;
  initial_guess.linear() =
      Eigen::AngleAxisd(angle, fodom::Vec3::UnitZ()).toRotationMatrix();
  initial_guess.translation() = fodom::Vec3(0.0, 0, 0);

  auto moving_cloud(cloud);

  // build fixed tree
  fodom::KdTreeConfig config;
  config.max_parallel_level = -1;
  fodom::PCloudType fixed_cloud = cloud;  // copy, constructor reorders
  auto fixed_tree = std::make_unique<fodom::TreeNode>(
      fixed_cloud.begin(), fixed_cloud.end(), config);

  auto t0 = std::chrono::high_resolution_clock::now();
  auto result =
      fodom::match_point_cloud(*fixed_tree, moving_cloud, initial_guess).pose;
  auto t1 = std::chrono::high_resolution_clock::now();
  std::cout << "ICP took "
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms\n";
  std::cout << "Result:\n"
            << "  R =\n"
            << result.rotation() << "\n"
            << "  t = " << result.translation().transpose() << "\n";

  double rot_err = Eigen::AngleAxisd(result.rotation()).angle() * 180.0 / M_PI;
  double trans_err = result.translation().norm();
  std::cout << "Error: rot=" << rot_err << " deg, trans=" << trans_err
            << " m\n\n";

  auto initial_cloud = transform_cloud(moving_cloud, initial_guess);
  auto result_cloud = transform_cloud(moving_cloud, result);

  // visualize
  polyscope::options::buildGui = false;
  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
  polyscope::init();

  g_state.pc_fixed =
      polyscope::registerPointCloud("fixed (ground truth)", fixed_cloud);
  g_state.pc_fixed->setPointRadius(g_state.point_radius);
  g_state.pc_fixed->setPointColor({0.2, 0.8, 0.2});  // green

  g_state.pc_initial =
      polyscope::registerPointCloud("initial guess", initial_cloud);
  g_state.pc_initial->setPointRadius(g_state.point_radius);
  g_state.pc_initial->setPointColor({0.8, 0.2, 0.2});  // red

  g_state.pc_result = polyscope::registerPointCloud("ICP result", result_cloud);
  g_state.pc_result->setPointRadius(g_state.point_radius);
  g_state.pc_result->setPointColor({0.2, 0.4, 0.9});  // blue

  std::cout << "Controls: F=toggle fixed, I=toggle initial, R=toggle result, "
               "+/-=point size\n";

  polyscope::state::userCallback = callback;
  polyscope::show();

  return 0;
}
