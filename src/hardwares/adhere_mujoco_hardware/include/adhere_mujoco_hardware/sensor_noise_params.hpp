#ifndef ADHERE_MUJOCO_HARDWARE__SENSOR_NOISE_PARAMS_HPP_
#define ADHERE_MUJOCO_HARDWARE__SENSOR_NOISE_PARAMS_HPP_

#include <mujoco/mujoco.h>

namespace adhere_mujoco_hardware
{

// Phase 3.5 realistic sensor noise/bias/decimation parameters (CLAUDE.md Phase 5.0 Stage 4):
// hardcoded compiled-in defaults here (bit-for-bit the original Phase 3.5 constexprs), each
// overridable per-model via an MJCF <custom><numeric name="sensor_noise_*"/> entry --
// AdhereMujocoHardware is a plain hardware_interface::SystemInterface, not an rclcpp::Node, so
// it has no ROS parameter access (the same reason HydroParams has an applyMjModelOverrides()
// hook instead of YAML -- see fossen_hydrodynamics.hpp), and this struct mirrors that exact
// pattern.
struct SensorNoiseParams
{
  double gyro_noise_std = 0.005;        // rad/s
  double gyro_bias_rw_std = 0.0005;     // rad/s, per-tick random-walk std
  double accel_noise_std = 0.02;        // m/s^2
  double att_noise_std = 0.003;         // rad, small-angle AHRS orientation noise
  double depth_noise_std = 0.005;       // m
  double dvl_vel_noise_std = 0.008;     // m/s
  double dvl_range_noise_std = 0.003;   // m
  double loadcell_noise_std = 0.15;     // N
  long long depth_decimation = 10;      // 500 / 10 = 50 Hz
  long long dvl_decimation = 25;        // 500 / 25 = 20 Hz

  // Scans m->nnumeric for <custom><numeric name="sensor_noise_..." data="..."/> entries by
  // name and overwrites the corresponding field in params if present, leaving params
  // completely untouched for any name absent from the MJCF (true for rov_wall.xml today --
  // zero regression). Mirrors HydroParams::applyMjModelOverrides() exactly.
  static void applyMjModelOverrides(const mjModel * m, SensorNoiseParams & params);
};

}  // namespace adhere_mujoco_hardware

#endif  // ADHERE_MUJOCO_HARDWARE__SENSOR_NOISE_PARAMS_HPP_
