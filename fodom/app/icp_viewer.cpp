#include <polyscope/point_cloud.h>
#include <polyscope/polyscope.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "fodom/icp.h"

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

struct CloudEntry {
  std::string name;
  polyscope::PointCloud* ps = nullptr;
};

struct State {
  fodom::PCloudType fixed_cloud;
  fodom::PCloudType moving_cloud;
  fodom::PCloudType aligned_cloud;

  std::unique_ptr<fodom::TreeNode> fixed_tree;
  fodom::KdTreeConfig kd_config;
  fodom::IcpConfig icp_config;

  fodom::PoseType result = fodom::PoseType::Identity();
  bool has_result = false;
  float point_radius = 0.002f;

  // 1=fixed, 2=moving, 3=aligned
  std::vector<CloudEntry> clouds;
};

static State g;

static void update_all_radii() {
  for (auto& c : g.clouds) {
    if (c.ps) c.ps->setPointRadius(g.point_radius);
  }
}

static void run_icp() {
  auto t0 = std::chrono::high_resolution_clock::now();

  g.result = fodom::match_point_cloud(*g.fixed_tree, g.moving_cloud,
                                      fodom::PoseType::Identity(), g.icp_config)
                 .pose;
  g.has_result = true;

  auto t1 = std::chrono::high_resolution_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  g.aligned_cloud = transform_cloud(g.moving_cloud, g.result);

  std::cout << "ICP done in " << ms << " ms\n";
  std::cout << "  t = " << g.result.translation().transpose() << "\n";
  Eigen::AngleAxisd aa(g.result.linear());
  std::cout << "  r = " << aa.angle() * 180.0 / M_PI << " deg\n";

  auto* pc = polyscope::registerPointCloud("aligned", g.aligned_cloud);
  pc->setPointRadius(g.point_radius);
  pc->setPointColor({0.1, 0.8, 0.1});

  // register in clouds list if not already there
  if (g.clouds.size() < 3) {
    g.clouds.push_back({"aligned", pc});
  } else {
    g.clouds[2].ps = pc;
  }
}

static void callback() {
  // point size: +/- keys
  if (ImGui::IsKeyPressed(ImGuiKey_Equal, true) ||
      ImGui::IsKeyPressed(ImGuiKey_KeypadAdd, true)) {
    g.point_radius = std::min(g.point_radius * 1.25f, 0.1f);
    update_all_radii();
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Minus, true) ||
      ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract, true)) {
    g.point_radius = std::max(g.point_radius / 1.25f, 0.0001f);
    update_all_radii();
  }

  // toggle clouds with 1, 2, 3
  if (ImGui::IsKeyPressed(ImGuiKey_1, false) && g.clouds.size() > 0 &&
      g.clouds[0].ps) {
    g.clouds[0].ps->setEnabled(!g.clouds[0].ps->isEnabled());
  }
  if (ImGui::IsKeyPressed(ImGuiKey_2, false) && g.clouds.size() > 1 &&
      g.clouds[1].ps) {
    g.clouds[1].ps->setEnabled(!g.clouds[1].ps->isEnabled());
  }
  if (ImGui::IsKeyPressed(ImGuiKey_3, false) && g.clouds.size() > 2 &&
      g.clouds[2].ps) {
    g.clouds[2].ps->setEnabled(!g.clouds[2].ps->isEnabled());
  }

  // ICP controls panel
  ImGui::SetNextWindowPos({10, 10}, ImGuiCond_Once);
  ImGui::SetNextWindowSize({300, 0}, ImGuiCond_Once);
  ImGui::Begin("ICP Controls");

  int n_iters = static_cast<int>(g.icp_config.n_iters);
  if (ImGui::SliderInt("Iterations", &n_iters, 1, 50))
    g.icp_config.n_iters = n_iters;

  float damping = static_cast<float>(g.icp_config.damping);
  if (ImGui::SliderFloat("Damping", &damping, 0.0f, 10.0f))
    g.icp_config.damping = damping;

  float max_dist = static_cast<float>(g.icp_config.max_correspondence_dist);
  if (ImGui::SliderFloat("Max corr. dist", &max_dist, 0.01f, 5.0f))
    g.icp_config.max_correspondence_dist = max_dist;

  if (ImGui::Button("Run ICP")) run_icp();

  if (g.has_result) {
    ImGui::Separator();
    auto t = g.result.translation();
    ImGui::Text("t: [%.3f, %.3f, %.3f]", t.x(), t.y(), t.z());
    Eigen::AngleAxisd aa(g.result.linear());
    ImGui::Text("r: %.2f deg", aa.angle() * 180.0 / M_PI);
  }

  ImGui::Separator();
  ImGui::TextDisabled("+/- point size | 1/2/3 toggle clouds");

  ImGui::End();
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: icp_viewer <fixed.bin> <moving.bin> "
              << "[n_iters] [max_corr_dist]\n";
    return 1;
  }

  g.fixed_cloud = load_cloud(argv[1]);
  g.moving_cloud = load_cloud(argv[2]);

  if (argc > 3) g.icp_config.n_iters = std::stoul(argv[3]);
  if (argc > 4) g.icp_config.max_correspondence_dist = std::stod(argv[4]);

  g.icp_config.damping = 0.0;

  std::cout << "Fixed:  " << g.fixed_cloud.size() << " points\n";
  std::cout << "Moving: " << g.moving_cloud.size() << " points\n";

  // build kd-tree on fixed cloud
  g.kd_config.max_parallel_level = 8;
  auto t0 = std::chrono::high_resolution_clock::now();
  g.fixed_tree = std::make_unique<fodom::TreeNode>(
      g.fixed_cloud.begin(), g.fixed_cloud.end(), g.kd_config);
  auto t1 = std::chrono::high_resolution_clock::now();
  std::cout << "KD-tree built in "
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms\n";

  polyscope::options::buildGui = false;
  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
  polyscope::init();

  auto* pc_fixed = polyscope::registerPointCloud("fixed", g.fixed_cloud);
  pc_fixed->setPointRadius(g.point_radius);
  pc_fixed->setPointColor({0.2, 0.4, 0.8});

  auto* pc_moving = polyscope::registerPointCloud("moving", g.moving_cloud);
  pc_moving->setPointRadius(g.point_radius);
  pc_moving->setPointColor({0.8, 0.3, 0.2});

  g.clouds.push_back({"fixed", pc_fixed});
  g.clouds.push_back({"moving", pc_moving});

  polyscope::state::userCallback = callback;
  polyscope::show();

  return 0;
}
