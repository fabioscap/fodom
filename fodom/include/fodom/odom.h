#include <fodom/imu.h>
#include <fodom/kdtree.h>

namespace fodom {

struct OdomConfig {
  KdTreeConfig kdtree;
  EkfConfig ekf;
  ICPConfig icp;
};

struct Odom {
  void process_imu(const ImuMeasurement& imu);
  void process_cloud(const PCloudType& cloud);

  OdomConfig config;

  double stamp;
};

}  // namespace fodom