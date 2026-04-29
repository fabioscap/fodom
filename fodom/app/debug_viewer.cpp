#include <polyscope/point_cloud.h>
#include <polyscope/polyscope.h>

#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "fodom/kdtree.h"

// ---- loaders ----

struct KeyframeLeaf {
  fodom::Vec3 mean;
  fodom::Vec3 normal;
};

static std::vector<KeyframeLeaf> load_keyframe_csv(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Cannot open: " + path);
  std::string line;
  std::getline(f, line);  // skip header
  std::vector<KeyframeLeaf> leaves;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ss(line);
    KeyframeLeaf lf;
    ss >> lf.mean.x() >> lf.mean.y() >> lf.mean.z();
    leaves.push_back(lf);
  }
  return leaves;
}

static fodom::PCloudType load_cloud_csv(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Cannot open: " + path);
  std::string line;
  std::getline(f, line);  // skip header
  fodom::PCloudType cloud;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ss(line);
    fodom::Vec3 p;
    ss >> p.x() >> p.y() >> p.z();
    cloud.push_back(p);
  }
  return cloud;
}

// Parse one Eigen-formatted row "[v0, v1, ...]" into doubles
static std::vector<double> parse_row(const std::string& line) {
  std::string s = line;
  s.erase(std::remove(s.begin(), s.end(), '['), s.end());
  s.erase(std::remove(s.begin(), s.end(), ']'), s.end());
  std::replace(s.begin(), s.end(), ',', ' ');
  std::istringstream ss(s);
  std::vector<double> vals;
  double v;
  while (ss >> v) vals.push_back(v);
  return vals;
}

struct DebugState {
  fodom::PoseType initial_guess;
  fodom::PoseType result;
  Eigen::Matrix<double, 6, 6> H;
  Eigen::Vector<double, 6> b;
};

static DebugState load_state(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Cannot open: " + path);

  std::map<std::string, std::vector<std::vector<double>>> sections;
  std::string cur;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    if (line.size() > 4 && line.substr(0, 4) == "=== ") {
      cur = line.substr(4, line.rfind(" ===") - 4);
    } else if (!line.empty() && line[0] == '[') {
      sections[cur].push_back(parse_row(line));
    }
  }

  auto parse_pose = [](const std::vector<std::vector<double>>& rows) {
    Eigen::Matrix4d m;
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c) m(r, c) = rows[r][c];
    fodom::PoseType pose;
    pose.matrix() = m;
    return pose;
  };

  DebugState s;
  s.initial_guess = parse_pose(sections["initial_guess"]);
  s.result = parse_pose(sections["result"]);
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c) s.H(r, c) = sections["H"][r][c];
  for (int r = 0; r < 6; ++r) s.b(r) = sections["b"][r][0];
  return s;
}

// ---- app state ----

static fodom::PCloudType transform_cloud(const fodom::PCloudType& cloud,
                                         const fodom::PoseType& T) {
  fodom::PCloudType out(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) out[i] = T * cloud[i];
  return out;
}

struct AppState {
  DebugState debug;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> t_eig;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> r_eig;

  float point_radius = 0.002f;
  polyscope::PointCloud* pc_kf = nullptr;
  polyscope::PointCloud* pc_moving = nullptr;
  polyscope::PointCloud* pc_initial = nullptr;
  polyscope::PointCloud* pc_result = nullptr;
};

static AppState g;

static void set_all_radii(float r) {
  for (auto* pc : {g.pc_kf, g.pc_moving, g.pc_initial, g.pc_result})
    if (pc) pc->setPointRadius(r);
}

static void callback() {
  if (ImGui::IsKeyPressed(ImGuiKey_Equal, true) ||
      ImGui::IsKeyPressed(ImGuiKey_KeypadAdd, true)) {
    g.point_radius = std::min(g.point_radius * 1.25f, 0.1f);
    set_all_radii(g.point_radius);
  }
  if (ImGui::IsKeyPressed(ImGuiKey_Minus, true) ||
      ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract, true)) {
    g.point_radius = std::max(g.point_radius / 1.25f, 0.0001f);
    set_all_radii(g.point_radius);
  }
  if (ImGui::IsKeyPressed(ImGuiKey_1, false) && g.pc_kf)
    g.pc_kf->setEnabled(!g.pc_kf->isEnabled());
  if (ImGui::IsKeyPressed(ImGuiKey_2, false) && g.pc_moving)
    g.pc_moving->setEnabled(!g.pc_moving->isEnabled());
  if (ImGui::IsKeyPressed(ImGuiKey_3, false) && g.pc_initial)
    g.pc_initial->setEnabled(!g.pc_initial->isEnabled());
  if (ImGui::IsKeyPressed(ImGuiKey_4, false) && g.pc_result)
    g.pc_result->setEnabled(!g.pc_result->isEnabled());

  ImGui::SetNextWindowPos({10, 10}, ImGuiCond_Once);
  ImGui::SetNextWindowSize({400, 0}, ImGuiCond_Once);
  ImGui::Begin("Debug State");

  auto& ig = g.debug.initial_guess;
  auto& res = g.debug.result;
  Eigen::AngleAxisd aa_ig(ig.linear()), aa_res(res.linear());
  auto ti = ig.translation(), tr = res.translation();
  ImGui::Text("Initial guess  t=[%.3f %.3f %.3f]  r=%.2f deg", ti.x(), ti.y(),
              ti.z(), aa_ig.angle() * 180.0 / M_PI);
  ImGui::Text("Result         t=[%.3f %.3f %.3f]  r=%.2f deg", tr.x(), tr.y(),
              tr.z(), aa_res.angle() * 180.0 / M_PI);

  ImGui::Separator();

  double t_det = g.debug.H.block<3, 3>(0, 0).determinant();
  double r_det = g.debug.H.block<3, 3>(3, 3).determinant();
  ImGui::Text("H_t  det=%.3e   eigs: %.3e  %.3e  %.3e", t_det,
              g.t_eig.eigenvalues()(0), g.t_eig.eigenvalues()(1),
              g.t_eig.eigenvalues()(2));
  ImGui::Text("H_r  det=%.3e   eigs: %.3e  %.3e  %.3e", r_det,
              g.r_eig.eigenvalues()(0), g.r_eig.eigenvalues()(1),
              g.r_eig.eigenvalues()(2));

  ImGui::Separator();

  auto& b = g.debug.b;
  ImGui::Text("b_t  [%.3e  %.3e  %.3e]", b(0), b(1), b(2));
  ImGui::Text("b_r  [%.3e  %.3e  %.3e]", b(3), b(4), b(5));

  ImGui::Separator();
  ImGui::TextDisabled(
      "1=keyframe(green)  2=moving(yellow)  3=initial(orange)  "
      "4=result(blue)  +/-=size");

  ImGui::End();
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: debug_viewer <base>  (e.g. /tmp/fodom_debug_0)\n";
    return 1;
  }

  const std::string base = argv[1];
  auto leaves = load_keyframe_csv(base + "_keyframe.csv");
  auto moving = load_cloud_csv(base + "_cloud.csv");
  g.debug = load_state(base + "_state.txt");
  g.t_eig.computeDirect(g.debug.H.block<3, 3>(0, 0));
  g.r_eig.computeDirect(g.debug.H.block<3, 3>(3, 3));

  fodom::PCloudType kf_means(leaves.size());
  for (size_t i = 0; i < leaves.size(); ++i) {
    kf_means[i] = leaves[i].mean;
  }

  auto initial_cloud = transform_cloud(moving, g.debug.initial_guess);
  auto result_cloud = transform_cloud(moving, g.debug.result);

  std::cout << "Keyframe leaves: " << leaves.size() << "\n";
  std::cout << "Moving points:   " << moving.size() << "\n";
  std::cout << "H_t det = " << g.debug.H.block<3, 3>(0, 0).determinant()
            << "\n";
  std::cout << "H_r det = " << g.debug.H.block<3, 3>(3, 3).determinant()
            << "\n";

  auto inspect_pose = [](const char* label, const fodom::PoseType& pose) {
    const auto& R = pose.linear();
    const auto& t = pose.translation();

    double s0 = R.col(0).norm();
    double s1 = R.col(1).norm();
    double s2 = R.col(2).norm();
    double det = R.determinant();
    Eigen::Vector3d euler = R.eulerAngles(2, 1, 0) * 180.0 / M_PI;

    std::cout << "\n=== " << label << " ===\n";
    std::cout << "matrix:\n" << pose.matrix() << "\n";
    std::cout << "t        = [" << t.transpose() << "]\n";
    std::cout << "euler(ZYX deg) = [" << euler.transpose() << "]\n";
    std::cout << "R col norms = " << s0 << "  " << s1 << "  " << s2 << "\n";
    std::cout << "R det    = " << det << "\n";
    if (std::abs(s0 - 1) > 1e-6 || std::abs(s1 - 1) > 1e-6 ||
        std::abs(s2 - 1) > 1e-6)
      std::cout << "*** SCALE DETECTED in R columns ***\n";
    if (std::abs(det - 1.0) > 1e-6)
      std::cout << "*** det(R) != 1: " << det << " ***\n";
  };

  inspect_pose("initial_guess", g.debug.initial_guess);
  inspect_pose("result", g.debug.result);

  polyscope::options::buildGui = false;
  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::None;
  polyscope::init();

  g.pc_kf = polyscope::registerPointCloud("keyframe", kf_means);
  g.pc_kf->setPointRadius(g.point_radius);
  g.pc_kf->setPointColor({0.2, 0.8, 0.2});

  g.pc_moving = polyscope::registerPointCloud("moving (raw)", moving);
  g.pc_moving->setPointRadius(g.point_radius);
  g.pc_moving->setPointColor({0.8, 0.8, 0.2});
  g.pc_moving->setEnabled(false);

  g.pc_initial = polyscope::registerPointCloud("initial guess", initial_cloud);
  g.pc_initial->setPointRadius(g.point_radius);
  g.pc_initial->setPointColor({0.9, 0.4, 0.1});

  g.pc_result = polyscope::registerPointCloud("result", result_cloud);
  g.pc_result->setPointRadius(g.point_radius);
  g.pc_result->setPointColor({0.2, 0.4, 0.9});

  polyscope::state::userCallback = callback;
  polyscope::show();

  return 0;
}
