#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("imu_noise_calib");

  const double duration = node->declare_parameter("duration", 10.0);
  const double g_mag = node->declare_parameter("gravity", 9.81);
  const double margin = node->declare_parameter("margin", 1.2);

  size_t n = 0;
  double sum_ax = 0, sum_ay = 0, sum_az = 0;
  double sum_gx = 0, sum_gy = 0, sum_gz = 0;
  double sumsq_a = 0;
  double sumsq_g = 0;

  auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(),
      [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
        const double ax = msg->linear_acceleration.x;
        const double ay = msg->linear_acceleration.y;
        const double az = msg->linear_acceleration.z;
        const double gx = msg->angular_velocity.x;
        const double gy = msg->angular_velocity.y;
        const double gz = msg->angular_velocity.z;
        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        sumsq_a += ax * ax + ay * ay + az * az;
        sumsq_g += gx * gx + gy * gy + gz * gz;
        ++n;
      });

  std::cerr << "Collecting IMU data for " << duration
            << " s — keep sensor still...\n";

  const auto t_start = node->now();
  while (rclcpp::ok() && (node->now() - t_start).seconds() < duration)
    rclcpp::spin_some(node);

  if (n < 2) {
    std::cerr << "Not enough samples (" << n
              << "). Is the IMU topic correct?\n";
    rclcpp::shutdown();
    return 1;
  }

  const double nd = static_cast<double>(n);
  const double ma_x = sum_ax / nd, ma_y = sum_ay / nd, ma_z = sum_az / nd;
  const double mg_x = sum_gx / nd, mg_y = sum_gy / nd, mg_z = sum_gz / nd;

  const double var_a = sumsq_a / nd - (ma_x * ma_x + ma_y * ma_y + ma_z * ma_z);
  const double var_g = sumsq_g / nd - (mg_x * mg_x + mg_y * mg_y + mg_z * mg_z);
  const double mean_a_norm = std::sqrt(ma_x * ma_x + ma_y * ma_y + ma_z * ma_z);
  const double accel_thr = std::abs(mean_a_norm - g_mag);
  const double gyro_thr = std::sqrt(mg_x * mg_x + mg_y * mg_y + mg_z * mg_z);

  std::cout << "\n--- measured (" << n << " samples) ---\n"
            << "  accel variance tr(Cov): " << var_a << " m^2/s^4\n"
            << "  gyro  variance tr(Cov): " << var_g << " rad^2/s^2\n"
            << "  |mean(|a|) - g|:        " << accel_thr << " m/s^2\n"
            << "  mean gyro magnitude:    " << gyro_thr << " rad/s\n"
            << "\n--- suggested config (" << margin << "x margin) ---\n"
            << "  ekf.still_accel_var: " << margin * var_a << "\n"
            << "  ekf.still_gyro_var:  " << margin * var_g << "\n"
            << "  ekf.still_accel_thr: " << margin * accel_thr << "\n"
            << "  ekf.still_gyro_thr:  " << margin * gyro_thr << "\n";

  rclcpp::shutdown();
  return 0;
}
