#ifndef ADHERE_CONTROLLER__DYNAMICS__MODELPARAMS_H_
#define ADHERE_CONTROLLER__DYNAMICS__MODELPARAMS_H_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"

#include "adhere_controller/algorithms/pid/AdherePidController.h"
#include "adhere_controller/algorithms/pid/SixDofPidController.h"
#include "adhere_controller/algorithms/smc/SixDofSmcController.h"
#include "adhere_controller/common/mathTypes.h"

namespace adhere_controller
{

// Runtime-loaded, model-decoupled physical/control parameters (CLAUDE.md Phase 3.3 Sec.3):
// everything here used to be a `constexpr`/`const Vec3` baked into
// StateOpenLoop.cpp/StatePidHold.cpp/ThrusterAllocator.h. Switching to a robot with different
// thruster geometry, mass, or gains now only requires editing
// `<pkg_description>/config/robot_control.yaml` -- zero adhere_controller C++ changes or
// recompilation.
struct ModelParams
{
  struct OpenLoopGains
  {
    // [Fx, Fy, Fz, Tx, Ty, Tz] max wrench magnitude at full stick deflection (CLAUDE.md
    // Phase 3.2 Stage 2.2.3). Only indices 0 (Fx), 1 (Fy), 2 (Fz), 5 (Tz) are consumed today.
    std::array<double, 6> max_wrench = {60.0, 50.0, 50.0, 5.0, 5.0, 8.0};
  };

  // Inherits SixDofPidController::Gains directly (CLAUDE.md Phase 5.0 Stage 2.4) instead of
  // mirroring its 6 fields + a hand-written copy function -- StatePidHold passes model_params_.
  // pid_hold straight to the SixDofPidController constructor via the base-class subobject.
  struct PidHoldGains : public SixDofPidController::Gains
  {
    PidHoldGains()
    {
      kp_pos = Vec3(120.0, 120.0, 120.0);
      ki_pos = Vec3(10.0, 10.0, 15.0);
      kd_vel = Vec3(60.0, 60.0, 60.0);
      kp_att = Vec3(40.0, 40.0, 25.0);
      ki_att = Vec3(2.0, 2.0, 2.0);
      kd_att = Vec3(15.0, 15.0, 12.0);
    }

    // [vx_max, vy_max, vz_max, omega_x_roll_max, omega_y_pitch_max, omega_z_yaw_max]
    // stick-driven setpoint-move rates (Phase 3.5: extended from 4 to 6 DOF).
    std::array<double, 6> max_teleop_vel = {0.25, 0.25, 0.25, 0.4, 0.4, 0.6};
    double max_tilt_deg = 45.0;  // Roll/pitch target clamp (CLAUDE.md Phase 3.5 Stage 2.3)
  };

  // Phase 3.8 wall-adhesion PID baseline (algorithms::pid::AdherePidController +
  // FSM::StateAdherePid): kp_pos/ki_pos/kd_vel/kp_att/ki_att/kd_att drive the X/Y/Z/attitude loop
  // (via the shared SixDofPidController) in both SOFT_DOCKING and WALL_ADHERED phases; the X
  // component of kp_pos/ki_pos/kd_vel is exercised only during SOFT_DOCKING in practice
  // (WALL_ADHERED phase re-latches p_d_world_.x every tick so the position loop's X contribution
  // stays ~0 there, and the dedicated force PI law below drives Fx instead) -- Y/Z are actively
  // regulated throughout (the STANDOFF_HOMING stage requires both to converge to the live
  // adhere_target_y/adhere_target_depth_m before SOFT_DOCKING/WALL_ADHERED begins).
  // approach_speed/dock_force_thresh/x_wall/x_dock_eps gate the SOFT_DOCKING->WALL_ADHERED
  // transition; kp_force/ki_force/kd_x/f_preload/f_des/integral_force_max are
  // AdherePidController's own wall-normal force-regulation gains (see its header for the sign
  // convention: positive magnitudes here, the minus sign that turns them into a pressing -- not
  // receding -- commanded wrench lives in code).
  // Inherits BOTH SixDofPidController::Gains (Y/Z/attitude loop) and AdherePidController::Gains
  // (wall-normal force PI loop) directly (CLAUDE.md Phase 5.0 Stage 2.4) instead of mirroring
  // their fields + two hand-written copy functions -- StateAdherePid passes model_params_.
  // adhere_pid straight to each controller's constructor via the respective base subobject.
  struct AdherePidGains : public SixDofPidController::Gains, public AdherePidController::Gains
  {
    AdherePidGains()
    {
      kp_pos = Vec3(120.0, 120.0, 120.0);
      ki_pos = Vec3(10.0, 10.0, 10.0);
      kd_vel = Vec3(60.0, 60.0, 60.0);
      kp_att = Vec3(40.0, 40.0, 25.0);
      ki_att = Vec3(2.0, 2.0, 2.0);
      kd_att = Vec3(15.0, 15.0, 12.0);
      // AdherePidController::Gains' own defaults (kp_force=2.0, ki_force=0.5, kd_x=20.0,
      // f_preload=30.0, integral_force_max=80.0) already match this struct's historical
      // defaults, so they are left un-reassigned here.
    }

    std::array<double, 6> max_teleop_vel = {0.25, 0.25, 0.25, 0.4, 0.4, 0.6};
    double max_tilt_deg = 30.0;
    double approach_speed = 0.15;       // m/s, x_d ramp rate toward x_wall while APPROACHing
    double dock_force_thresh = 20.0;    // N, sum F_N,i required to leave APPROACH
    double x_wall = 0.0;                // m, world X of the wall plane
    double x_dock_eps = 0.02;           // m, position tolerance for the APPROACH->ADHERED gate
    double f_des = 100.0;               // N, desired sum of the 4 pad normal forces
  };

  // Phase 3.8 wall-adhesion SMC baseline (algorithms::smc::SixDofSmcController +
  // FSM::StateAdhereSmc): lambda/kd/k_switch/phi are the 6-DOF sliding-surface slope/damping/
  // switching-gain/boundary-layer vectors (order [x,y,z,roll,pitch,yaw], body frame).
  // wall_preload_fx is a constant pressing bias (N magnitude; applied as a negative Fx wrench)
  // added only once StateAdhereSmc has docked -- see its header for why a single
  // SixDofSmcController instance drives both APPROACH and ADHERED phases.
  // Inherits SixDofSmcController::Gains directly (CLAUDE.md Phase 5.0 Stage 2.4) instead of
  // mirroring its 6 fields + a hand-written copy function.
  struct AdhereSmcGains : public SixDofSmcController::Gains
  {
    AdhereSmcGains()
    {
      lambda = (Vec6() << 2.0, 2.0, 2.0, 1.5, 1.5, 1.5).finished();
      kd = (Vec6() << 80.0, 80.0, 80.0, 20.0, 20.0, 20.0).finished();
      k_switch = (Vec6() << 40.0, 40.0, 40.0, 10.0, 10.0, 10.0).finished();
      phi = (Vec6() << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05).finished();
      // This phase: bounded integral-sliding-mode augmentation (s = e_nu + lambda*e_eta +
      // ki*int(e_eta)dt), added because a pure P+switching surface has nothing to "ratchet up"
      // against static Coulomb pad friction once wall preload is pressing -- defaults to
      // all-zero so any other future SMC consumer is unaffected; only adhere_smc.ki in
      // robot_control.yaml sets the Y/Z components nonzero.
      ki = (Vec6() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
      wall_preload_fx = 30.0;  // N magnitude; applied as -wall_preload_fx wrench
    }

    std::array<double, 6> max_teleop_vel = {0.25, 0.25, 0.25, 0.4, 0.4, 0.6};
    double max_tilt_deg = 30.0;
    double approach_speed = 0.15;
    double dock_force_thresh = 20.0;
    double x_wall = 0.0;
    double x_dock_eps = 0.02;
  };

  // Phase 4.2 wall-homing/adhesion sequence (renamed/extended this phase to 3 stages), shared by
  // ADHERE_PID_WBC/ADHERE_SMC_WBC/ADHERE_MPPI_BASELINE: on entry, each first stabilizes BOTH
  // lateral and depth position at a safe standoff distance from the wall (STANDOFF_HOMING)
  // before approaching (SOFT_DOCKING) and adhering (WALL_ADHERED). Deliberately a single shared
  // block (not duplicated per-algorithm like AdherePidGains/AdhereSmcGains) -- the point is one
  // zero-recompile target depth/standoff shared by all three baselines.
  struct AdhereStageParams
  {
    double target_depth_m = 1.0;  // m, Z_target = -target_depth_m
    double target_y = 0.0;        // m, world Y
    double standoff_x = 0.35;     // m, STANDOFF_HOMING holds this far from the wall plane (X=0)
    // This phase: STANDOFF_HOMING -> SOFT_DOCKING gate tightened from Z-only to Y+Z+planar-speed
    // (previously named DEPTH_ALIGN -> WALL_ADHERE and gated on Z alone -- renamed/extended so
    // lateral homing completes at a friction-free standoff distance *before* any wall preload
    // engages, instead of relying on the Y-PID/SMC surface to fight Coulomb pad friction after
    // docking).
    double depth_tol = 0.03;      // m, |z - Z_target| gate for STANDOFF_HOMING -> SOFT_DOCKING
    double y_tol = 0.03;          // m, |y - Y_target| gate for STANDOFF_HOMING -> SOFT_DOCKING
    double vel_tol = 0.04;        // m/s, sqrt(vy^2+vz^2) planar-speed gate for STANDOFF_HOMING
    // Phase 4.3 soft-landing approach damping: ADHERE_PID_WBC/ADHERE_SMC_WBC subtract
    // approach_vx_damp*max(0, v_x,body) from the commanded Fx during the pre-dock SOFT_DOCKING
    // ramp (!docked_) so the ROV decelerates into the wall instead of impacting at the ramp's
    // constant approach_speed. Not applied once docked_ (the force PI/SMC preload law owns Fx
    // from then on).
    double approach_vx_damp = 80.0;  // N/(m/s)
    // This phase: 0.06m auto-detach threshold -- if the live wall target moves by more than this
    // on either Y or Z while docked, the ADHERE_* state re-enters STANDOFF_HOMING instead of
    // dragging itself sideways/vertically while still pressed against the wall.
    double retarget_threshold = 0.06;  // m
  };

  // This phase: wall QR-code/AprilTag preset table (robot_control.yaml's adhere_wall_tags_y /
  // adhere_wall_tags_depth_m parallel flat arrays, zipped by index -- ROS parameters have no
  // array-of-struct type). Resolved by AdhereController's /rov/adhere_target_cmd subscription
  // callback when command_type == TAG_PRESET.
  struct AdhereWallTag
  {
    double y = 0.0;
    double depth_m = 1.0;
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

  // Phase 5.0 Stage 4: RovStateEstimator's fixed correction gains, moved out of that header's
  // `static constexpr` into YAML (CLAUDE.md rationale: unlike AdhereMujocoHardware's sim-
  // injected sensor noise std -- a plain SystemInterface with no ROS parameter access, hence
  // MJCF-sourced -- RovStateEstimator runs inside the adhere_controller ROS node, which CAN read
  // robot_control.yaml; these gains are measured/tuned from a real sensor suite in a real
  // experiment, not simulation-only). Defaults match the original Phase 3.5/3.7 compiled-in
  // values exactly, verified unchanged against the <0.015 m/s / <15mm / <1.0deg RMSE contract in
  // test_sensor_fusion_and_teleop.cpp.
  struct EstimatorGains
  {
    double att_gain = 0.30;
    double depth_pos_gain = 0.30;
    double depth_vel_gain = 0.10;
    double dvl_vel_gain = 0.50;
    double dvl_range_gain = 0.30;
    double contact_force_filter_gain = 0.20;
    double contact_vel_gain = 0.50;
    double contact_pos_gain = 0.20;
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

  OpenLoopGains open_loop;
  PidHoldGains pid_hold;
  AdherePidGains adhere_pid;
  AdhereSmcGains adhere_smc;
  AdhereStageParams adhere_stage;
  // Defaults match this phase's task spec: Tag0 (0,-1.0), Tag1 (+0.8,-1.0), Tag2 (-0.8,-1.5),
  // Tag3 (+1.2,-2.0).
  std::vector<AdhereWallTag> adhere_wall_tags = {
    {0.00, 1.00}, {0.80, 1.00}, {-0.80, 1.50}, {1.20, 2.00}};
  SensorMount sensor_mount;
  DynamicsParams dynamics;
  EstimatorGains estimator;

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
