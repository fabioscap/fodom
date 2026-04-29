#include <polyscope/point_cloud.h>
#include <polyscope/polyscope.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <random>
#include <set>
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

// generate a random RGB color from a node index
static std::array<double, 3> random_color(size_t seed) {
  std::mt19937 rng(seed * 2654435761u);
  std::uniform_real_distribution<double> dist(0.2, 1.0);
  return {dist(rng), dist(rng), dist(rng)};
}

// walk the tree and assign per-point colors for each level
static void collect_level_colors(
    const fodom::TreeNode& node, size_t start, size_t depth, size_t max_depth,
    size_t node_id,
    std::vector<std::vector<std::array<double, 3>>>& level_colors) {
  const size_t n = node.num_points;

  auto color = random_color(node_id);
  for (size_t lvl = depth; lvl <= max_depth; ++lvl) {
    for (size_t i = start; i < start + n; ++i) {
      level_colors[lvl][i] = color;
    }
  }

  size_t split = 0;
  if (node.left && node.right)
    split = node.left->num_points;
  else if (node.left)
    split = node.left->num_points;
  else if (node.right)
    split = n - node.right->num_points;

  size_t child_id = node_id * 2 + 1;
  if (node.left) {
    collect_level_colors(*node.left, start, depth + 1, max_depth, child_id,
                         level_colors);
  }
  if (node.right) {
    collect_level_colors(*node.right, start + split, depth + 1, max_depth,
                         child_id + 1, level_colors);
  }
}

static size_t tree_depth(const fodom::TreeNode& node) {
  size_t d = 0;
  if (node.left) d = std::max(d, 1 + tree_depth(*node.left));
  if (node.right) d = std::max(d, 1 + tree_depth(*node.right));
  return d;
}

struct State {
  polyscope::PointCloud* pc = nullptr;
  const fodom::PCloudType* cloud = nullptr;
  float point_radius = 0.002f;
  int current_level = -1;  // -1 = no tree coloring
  int max_level = 0;
  std::vector<std::vector<std::array<double, 3>>> level_colors;
};

static State g_state;

static void apply_level_coloring() {
  if (g_state.current_level < 0) {
    g_state.pc->removeAllQuantities();
    std::cout << "Tree coloring off\n";
    return;
  }

  size_t lvl = static_cast<size_t>(g_state.current_level);
  g_state.pc->addColorQuantity("tree_level", g_state.level_colors[lvl])
      ->setEnabled(true);
  std::cout << "Level " << lvl << " / " << g_state.max_level << "\n";
}

static void callback() {
  // tree level: [ and ] to cycle, 0 to reset
  if (ImGui::IsKeyPressed(ImGuiKey_RightBracket, false)) {
    if (g_state.current_level < g_state.max_level) g_state.current_level++;
    apply_level_coloring();
  }
  if (ImGui::IsKeyPressed(ImGuiKey_LeftBracket, false)) {
    g_state.current_level--;
    if (g_state.current_level < -1) g_state.current_level = -1;
    apply_level_coloring();
  }
  if (ImGui::IsKeyPressed(ImGuiKey_0, false)) {
    g_state.current_level = -1;
    apply_level_coloring();
  }

  // point size: +/- keys
  if (ImGui::IsKeyPressed(ImGuiKey_Equal, true) ||
      ImGui::IsKeyPressed(ImGuiKey_KeypadAdd, true)) {
    g_state.point_radius = std::min(g_state.point_radius * 1.25f, 0.1f);
    g_state.pc->setPointRadius(g_state.point_radius);
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Minus, true) ||
      ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract, true)) {
    g_state.point_radius = std::max(g_state.point_radius / 1.25f, 0.0001f);
    g_state.pc->setPointRadius(g_state.point_radius);
  }
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: visualize <cloud.bin>\n";
    return 1;
  }

  auto cloud = load_cloud(argv[1]);
  // filter NaN/Inf points (common in LiDAR data)
  cloud.erase(
      std::remove_if(cloud.begin(), cloud.end(),
                     [](const fodom::Vec3& p) { return !p.allFinite(); }),
      cloud.end());
  std::cout << "Loaded " << cloud.size() << " points (after NaN filter)\n";

  fodom::KdTreeConfig config;
  config.max_parallel_level = 8;
  auto t0 = std::chrono::high_resolution_clock::now();
  auto tree =
      std::make_unique<fodom::TreeNode>(cloud.begin(), cloud.end(), config);
  auto t1 = std::chrono::high_resolution_clock::now();
  std::cout << "KD-tree built in "
            << std::chrono::duration<double, std::milli>(t1 - t0).count()
            << " ms\n";

  // precompute per-level colors
  size_t depth = tree_depth(*tree);
  g_state.max_level = static_cast<int>(depth);
  g_state.level_colors.resize(depth + 1, std::vector<std::array<double, 3>>(
                                             cloud.size(), {0.6, 0.6, 0.6}));
  collect_level_colors(*tree, 0, 0, depth, 0, g_state.level_colors);
  std::cout << "Tree depth: " << depth << "\n";

  // diagnostic: print how many distinct colors per level
  for (size_t lvl = 0; lvl <= depth; ++lvl) {
    std::set<std::array<double, 3>> unique;
    for (auto& c : g_state.level_colors[lvl]) unique.insert(c);
    std::cout << "  level " << lvl << ": " << unique.size() << " regions\n";
  }

  polyscope::options::buildGui = false;
  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
  polyscope::init();

  g_state.cloud = &cloud;
  g_state.pc = polyscope::registerPointCloud("cloud", cloud);
  g_state.pc->setPointRadius(g_state.point_radius);

  polyscope::state::userCallback = callback;
  polyscope::show();

  return 0;
}
