#include "adhere_mujoco_hardware/sensor_noise_params.hpp"

namespace adhere_mujoco_hardware
{

void SensorNoiseParams::applyMjModelOverrides(const mjModel * m, SensorNoiseParams & params)
{
  // Looks up a <custom><numeric name="name" data="..."/> entry by name; returns nullptr (and
  // leaves the corresponding field untouched) if the MJCF declares no numeric of that name --
  // true for every name below on rov_wall.xml today, which is exactly the required zero-
  // regression default.
  auto findNumeric = [m](const char * name, int min_size) -> const double * {
    const int id = mj_name2id(m, mjOBJ_NUMERIC, name);
    if (id < 0 || m->numeric_size[id] < min_size) {
      return nullptr;
    }
    return m->numeric_data + m->numeric_adr[id];
  };
  auto scalarOverride = [&](const char * name, double & field) {
    if (const double * d = findNumeric(name, 1)) {
      field = d[0];
    }
  };
  auto decimationOverride = [&](const char * name, long long & field) {
    if (const double * d = findNumeric(name, 1)) {
      field = static_cast<long long>(d[0]);
    }
  };

  scalarOverride("sensor_noise_gyro_std", params.gyro_noise_std);
  scalarOverride("sensor_noise_gyro_bias_rw_std", params.gyro_bias_rw_std);
  scalarOverride("sensor_noise_accel_std", params.accel_noise_std);
  scalarOverride("sensor_noise_att_std", params.att_noise_std);
  scalarOverride("sensor_noise_depth_std", params.depth_noise_std);
  scalarOverride("sensor_noise_dvl_vel_std", params.dvl_vel_noise_std);
  scalarOverride("sensor_noise_dvl_range_std", params.dvl_range_noise_std);
  scalarOverride("sensor_noise_loadcell_std", params.loadcell_noise_std);
  decimationOverride("sensor_noise_depth_decimation", params.depth_decimation);
  decimationOverride("sensor_noise_dvl_decimation", params.dvl_decimation);
}

}  // namespace adhere_mujoco_hardware
