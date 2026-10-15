#ifndef ADHERE_MUJOCO_HARDWARE__FOSSEN_HYDRODYNAMICS_HPP_
#define ADHERE_MUJOCO_HARDWARE__FOSSEN_HYDRODYNAMICS_HPP_

#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace adhere_mujoco_hardware
{

// Half-sine velocity pulse in the world frame (e.g. a wave-impact gust). Peaks at the
// midpoint of [t_start, t_end] and tapers to zero at both edges. Exact port of
// mujoco_mppi_ws/src/env/hydrodynamics.py::GustEvent.
struct GustEvent
{
  double t_start = 0.0;
  double t_end = 0.0;
  double peak_y = 0.0;  // m/s, world +Y
  double peak_x = 0.0;  // m/s, world +X (normal "suction" component)

  Eigen::Vector3d velocity(double t) const
  {
    if (t < t_start || t > t_end) {
      return Eigen::Vector3d::Zero();
    }
    const double phase = (t - t_start) / (t_end - t_start) * M_PI;
    const double s = std::sin(phase);
    return Eigen::Vector3d(peak_x * s, peak_y * s, 0.0);
  }
};

// Exact port of mujoco_mppi_ws/src/env/hydrodynamics.py::HydroParams. Field defaults match
// the Python calm-water baseline bit-for-bit.
struct HydroParams
{
  double rho_water = 1000.0;      // kg/m^3
  double gravity = 9.81;          // m/s^2
  double vehicle_mass = 20.0;     // kg, dry mass
  double augmented_mass = 26.0;   // kg, MJCF <inertial> mass (dry + added mass)
  double displaced_volume = (1.01 * 20.0) / 1000.0;  // m^3, 1% net positive buoyancy

  Eigen::Vector3d cob_offset_body = Eigen::Vector3d(0.0, 0.0, 0.05);

  double z_surface = 1.0;      // m
  double z_transition = 0.4;   // m

  Eigen::Vector3d D_lin = Eigen::Vector3d(30.0, 20.0, 25.0);
  Eigen::Vector3d D_quad = Eigen::Vector3d(140.0, 50.0, 70.0);
  Eigen::Vector3d D_ang_lin = Eigen::Vector3d(8.0, 8.0, 10.0);
  Eigen::Vector3d D_ang_quad = Eigen::Vector3d(15.0, 15.0, 20.0);

  Eigen::Vector3d current_steady_world = Eigen::Vector3d::Zero();
  std::vector<GustEvent> gust_events;

  double C_venturi = 0.45;
  double h_gap_nominal = 0.025;  // m
  double A_plate = 0.24;         // m^2
  double h0_decay = 0.05;        // m
  double wall_x_offset = 0.1;    // m

  // Ambient current velocity in the world frame at time t: steady crossflow + superposed
  // wave-gust pulses.
  Eigen::Vector3d currentWorld(double t) const
  {
    Eigen::Vector3d v = current_steady_world;
    for (const auto & gust : gust_events) {
      v += gust.velocity(t);
    }
    return v;
  }
};

// Rigid-body kinematic state needed to evaluate the hydrodynamic wrench, expressed exactly
// as MuJoCo exposes it for a free-jointed body: world-frame CoM position/orientation,
// world-frame angular velocity (as in mjData::cvel[0:3]) and world-frame CoM linear
// velocity (as in mjData::cvel[3:6]).
struct RigidBodyState
{
  Eigen::Vector3d com_pos_world = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // body -> world
  Eigen::Vector3d ang_vel_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d lin_vel_world = Eigen::Vector3d::Zero();
};

struct Wrench
{
  Eigen::Vector3d force_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d torque_world = Eigen::Vector3d::Zero();
};

// Exact C++ port of mujoco_mppi_ws/src/env/hydrodynamics.py::setup_hydrodynamics_callback's
// per-tick passive_callback body. Stateless/pure: all per-step quantities are passed in, so
// the same implementation can be shared by AdhereMujocoHardware (at 500 Hz) and any future
// rollout/controller code without duplicating the physics.
class FossenHydrodynamics
{
public:
  // Returns the total hydrodynamic force/torque (world frame, applied at the body CoM) for
  // the given rigid-body state, simulation time t, and hydrodynamic parameters.
  static Wrench compute(const RigidBodyState & state, double t, const HydroParams & params);
};

}  // namespace adhere_mujoco_hardware

#endif  // ADHERE_MUJOCO_HARDWARE__FOSSEN_HYDRODYNAMICS_HPP_
