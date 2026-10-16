#ifndef ADHERE_MUJOCO_HARDWARE__FLOW_ENVIRONMENT_PLUGIN_HPP_
#define ADHERE_MUJOCO_HARDWARE__FLOW_ENVIRONMENT_PLUGIN_HPP_

#include <memory>
#include <string>

#include "adhere_mujoco_hardware/fossen_hydrodynamics.hpp"

namespace adhere_mujoco_hardware
{

// Strategy interface for the ambient wave/flow environment: each concrete plugin supplies a
// fully-configured HydroParams (so AdhereMujocoHardware/FossenHydrodynamics never need to
// know which profile is active) plus a convenience accessor for the instantaneous ambient
// flow velocity, used for the /rov/environment_markers flow-vector arrow.
class FlowEnvironmentPlugin
{
public:
  virtual ~FlowEnvironmentPlugin() = default;
  virtual const HydroParams & getHydroParams() const = 0;
  virtual Eigen::Vector3d ambientVelocityWorld(double t) const
  {
    return getHydroParams().currentWorld(t);
  }
  virtual Eigen::Vector3d ambientAccelerationWorld(double t) const
  {
    return getHydroParams().currentAccelerationWorld(t);
  }
  virtual std::string profileName() const = 0;
};

// Default profile: still water. Ambient flow velocity/acceleration are identically zero;
// the ROV is subject only to added mass, Morison drag, Venturi suction, and the buoyancy/
// gravity static bias.
class CalmWaterPlugin : public FlowEnvironmentPlugin
{
public:
  const HydroParams & getHydroParams() const override { return params_; }
  std::string profileName() const override { return "calm"; }

private:
  HydroParams params_{};  // all defaults == calm water
};

// Station-keeping + wave-impact gust preset, exact port of
// mujoco_mppi_ws/src/env/hydrodynamics.py::station_keeping_hydro_params(): a steady 0.3 m/s
// World-Y crossflow plus two half-sine wave gusts at t in [3.0, 4.2]s and [7.0, 8.2]s (peak
// 0.9 m/s along Y, 0.2 m/s normal-suction component along X). All other HydroParams fields
// keep their calm-water defaults.
class StationKeepingGustPlugin : public FlowEnvironmentPlugin
{
public:
  StationKeepingGustPlugin()
  {
    params_.current_steady_world = Eigen::Vector3d(0.0, 0.3, 0.0);
    params_.gust_events = {
      GustEvent{3.0, 4.2, 0.9, 0.2},
      GustEvent{7.0, 8.2, 0.9, 0.2},
    };
  }

  const HydroParams & getHydroParams() const override { return params_; }
  std::string profileName() const override { return "station_keeping_gust"; }

private:
  HydroParams params_{};
};

// Factory: selects a concrete plugin by the `wave_profile` hardware_interface param. Unknown
// profile names fall back to calm water (documented, not silently mismatched).
std::unique_ptr<FlowEnvironmentPlugin> createFlowEnvironment(const std::string & profile_name);

}  // namespace adhere_mujoco_hardware

#endif  // ADHERE_MUJOCO_HARDWARE__FLOW_ENVIRONMENT_PLUGIN_HPP_
