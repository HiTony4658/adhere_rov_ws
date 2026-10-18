#ifndef ADHERE_CONTROLLER__DYNAMICS__MODELPARAMS_H_
#define ADHERE_CONTROLLER__DYNAMICS__MODELPARAMS_H_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"

#include "adhere_controller/common/mathTypes.h"

namespace adhere_controller
{

// Runtime-loaded, model-decoupled physical/control parameters (CLAUDE.md Phase 3.3 Sec.3):
// everything here used to be a `constexpr`/`const Vec3` baked into StateDepthHold.cpp/
// StateOpenLoop.cpp/StatePidHold.cpp/ThrusterAllocator.h. Switching to a robot with different
// thruster geometry, mass, or gains now only requires editing
// `<pkg_description>/config/robot_control.yaml` -- zero adhere_controller C++ changes or
// recompilation.
struct ModelParams
{
  struct DepthHoldGains
  {
    double kp_z = 180.0;
    double ki_z = 95.0;
    double kd_z = 62.0;
    double max_heave_vel = 0.30;  // m/s, Xbox left-stick full-deflection target_z_ rate
    double kp_att = 6.0;
    double kd_att = 4.0;
  };

  struct OpenLoopGains
  {
    // [Fx, Fy, Fz, Tx, Ty, Tz] max wrench magnitude at full stick deflection (CLAUDE.md
    // Phase 3.2 Stage 2.2.3). Only indices 0 (Fx), 1 (Fy), 2 (Fz), 5 (Tz) are consumed today.
    std::array<double, 6> max_wrench = {60.0, 50.0, 50.0, 5.0, 5.0, 8.0};
  };

  struct PidHoldGains
  {
    Vec3 kp_pos = Vec3(120.0, 120.0, 120.0);
    Vec3 ki_pos = Vec3(10.0, 10.0, 15.0);
    Vec3 kd_vel = Vec3(60.0, 60.0, 60.0);
    Vec3 kp_att = Vec3(40.0, 40.0, 25.0);
    Vec3 ki_att = Vec3(2.0, 2.0, 2.0);
    Vec3 kd_att = Vec3(15.0, 15.0, 12.0);
    // [vx_max, vy_max, vz_max, omega_x_roll_max, omega_y_pitch_max, omega_z_yaw_max]
    // stick-driven setpoint-move rates (Phase 3.5: extended from 4 to 6 DOF).
    std::array<double, 6> max_teleop_vel = {0.25, 0.25, 0.25, 0.4, 0.4, 0.6};
    double max_tilt_deg = 45.0;  // Roll/pitch target clamp (CLAUDE.md Phase 3.5 Stage 2.3)
  };

  // Phase 3.5 sensor mounting lever arms (body frame, relative to CoM), used by
  // RovStateEstimator's lever-arm compensation formulas. Must match mjcf/rov_wall.xml's
  // imu_site/depth_site/dvl_site positions.
  struct SensorMount
  {
    Vec3 r_imu = Vec3(0.0, 0.0, 0.0);
    Vec3 r_depth = Vec3(0.0, 0.0, 0.0);
    Vec3 r_dvl = Vec3(0.0, 0.0, -0.20);
  };

  // Phase 3.7 nominal Fossen rigid-body + added-mass diagonal terms (dynamics::FossenDynamics):
  // CoM at body origin, principal-axis inertia assumed (matches the legacy
  // ~/Research/mujoco_mppi_ws/src/env/hydrodynamics.py reference's own no-off-diagonal-coupling
  // treatment). `augmented_mass` above is kept untouched for backward compat; these are the
  // actual per-DOF terms FossenDynamics::massMatrix() assembles.
  struct DynamicsParams
  {
    Vec3 inertia_diag = Vec3(0.40, 0.90, 0.90);        // kg*m^2, [Ixx, Iyy, Izz]
    Vec3 added_mass_lin = Vec3(6.0, 6.0, 6.0);         // kg, surge/sway/heave added mass
    Vec3 added_mass_ang = Vec3(0.05, 0.10, 0.10);      // kg*m^2, roll/pitch/yaw added inertia
  };

  // Thruster geometry/limits (body frame, relative to CoM) for dynamically building
  // B in R^{6x8} / B^+ in R^{8x6} (allocation::ThrusterAllocator.h). Default values are
  // rov_wall.xml's REAL geometry (thr_N1..N4 normal thrusters at the 4 corners, gear
  // "-1 0 0 0 0 0"; thr_T5..T8 tangential thrusters at the same 4 corners, direction from each
  // site's zaxis) -- see CLAUDE.md Phase 3.2's documented deviation from Sec.4.3's illustrative
  // lever-arm placeholder.
  std::array<Vec3, 8> thruster_pos = {
    Vec3(0.0, 0.25, 0.15), Vec3(0.0, -0.25, 0.15), Vec3(0.0, 0.25, -0.15),
    Vec3(0.0, -0.25, -0.15), Vec3(0.0, 0.25, 0.15), Vec3(0.0, -0.25, 0.15),
    Vec3(0.0, 0.25, -0.15), Vec3(0.0, -0.25, -0.15)};
  std::array<Vec3, 8> thruster_dir = {
    Vec3(-1.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0),
    Vec3(0.0, -0.8660254037844386, 0.5), Vec3(0.0, 0.8660254037844386, 0.5),
    Vec3(0.0, -0.8660254037844386, -0.5), Vec3(0.0, 0.8660254037844386, -0.5)};
  std::array<double, 8> thruster_min = {-40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0};
  std::array<double, 8> thruster_max = {40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0};

  // Contact pad geometry (body frame), for Rhombic Kern / future WBC. Unused by the current
  // FSM states.
  std::array<Vec3, 4> pad_pos = {
    Vec3(-0.10, 0.25, 0.15), Vec3(-0.10, -0.25, 0.15), Vec3(-0.10, 0.25, -0.15),
    Vec3(-0.10, -0.25, -0.15)};
  // Rhombic Kern safety margin (CLAUDE.md Sec.4.4/Sec.5, Phase 3.6): shrinks the admissible
  // kern region C_kern*u<=0 enforces by this fraction. Not yet consumed by any live constraint
  // (that lands with Phase 4's 12D OSQP) -- precomputed now purely for AdhereController's
  // estimated kern_margin telemetry on /rov/controller_state.
  double kern_eps_margin = 0.20;

  // Hydrostatic & Fossen hydrodynamic feedforward parameters.
  double rho_water = 1000.0;
  double gravity = 9.81;
  double vehicle_mass = 20.0;
  double augmented_mass = 26.0;
  double displaced_volume = 0.0202;
  Vec3 cob_offset_body = Vec3(0.0, 0.0, 0.05);
  double z_surface = 0.0;
  double z_transition = 0.4;
  Vec3 D_lin = Vec3(30.0, 20.0, 25.0);
  Vec3 D_quad = Vec3(140.0, 50.0, 70.0);
  Vec3 D_ang_lin = Vec3(8.0, 8.0, 10.0);
  Vec3 D_ang_quad = Vec3(15.0, 15.0, 20.0);

  // Workspace safety bounds (world frame).
  double workspace_z_min = -9.80;
  double workspace_z_max = -0.15;
  double workspace_x_min = 0.115;

  // Phase 3.7 contact-aided estimator update (RovStateEstimator::correctContact): a wall-contact
  // pseudo-measurement fires only once all 4 filtered pad load cells exceed this threshold.
  double contact_force_thresh = 2.0;  // N

  DepthHoldGains depth_hold;
  OpenLoopGains open_loop;
  PidHoldGains pid_hold;
  SensorMount sensor_mount;
  DynamicsParams dynamics;

  // Phase 3.5 dual state-acquisition mode: "ground_truth" or "sensor_fusion" (CLAUDE.md
  // Stage 2.3). Read directly by AdhereController, not by any FSMState.
  std::string estimation_mode = "ground_truth";

  // Declares/reads every field above as a ROS parameter on `node` (mirrors the existing
  // auto_declare<int>("cm_update_rate", ...) pattern already used for the multi-rate
  // parameters in AdhereController::on_configure()). Flat vector/scalar parameters are
  // declared with this struct's compiled-in defaults as their ROS defaults, so a
  // robot_control.yaml that omits a key still yields a fully-specified ModelParams. Defined in
  // dynamics/ModelParams.cpp.
  static ModelParams loadFromNode(const rclcpp::Node::SharedPtr & node);
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__DYNAMICS__MODELPARAMS_H_
