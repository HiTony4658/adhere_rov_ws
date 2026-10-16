#ifndef ADHERE_MUJOCO_HARDWARE__MUJOCO_VIS_BRIDGE_HPP_
#define ADHERE_MUJOCO_HARDWARE__MUJOCO_VIS_BRIDGE_HPP_

#include <array>
#include <atomic>
#include <mutex>
#include <vector>

namespace adhere_mujoco_hardware
{

// In-process 500Hz<->60Hz snapshot bridge between AdhereMujocoHardware (producer) and
// MujocoViewer/adhere_mujoco_node (consumers). Two different synchronization styles are used
// for the two directions of traffic:
//   - Bulk hardware->GUI telemetry (qpos/qvel/ctrl/pad_force/flow_vel_world) is copied under
//     `mtx` with a single try_lock() per 500Hz tick (CLAUDE.md Phase 2.6 Stage 3.2) so a
//     stalled renderer can never block the real-time write() loop.
//   - Scalar control knobs that flow in either direction (GUI->hardware: manual thruster
//     sliders, perturbation wrench, run/pause, reset, wave-mode request; hardware->GUI:
//     measured physics rate) are plain lock-free std::atomic members, safe to touch from any
//     thread without taking `mtx`.
struct MujocoVisBridge
{
  explicit MujocoVisBridge(int nq, int nv)
  : qpos(static_cast<size_t>(nq), 0.0), qvel(static_cast<size_t>(nv), 0.0)
  {
    for (auto & v : manual_ctrl) {
      v.store(0.0);
    }
    for (auto & v : xfrc_perturb) {
      v.store(0.0);
    }
  }

  // --- mutex-protected bulk snapshot (500Hz producer, ~60Hz/100Hz consumers) ---
  std::mutex mtx;
  std::vector<double> qpos;
  std::vector<double> qvel;
  std::array<double, 8> ctrl{};
  std::array<double, 4> pad_force{};       // [TL, TR, BL, BR], N
  std::array<double, 4> pad_v_sep{};       // [TL, TR, BL, BR], m/s
  std::array<double, 3> flow_vel_world{};  // ambient flow velocity, world frame, m/s
  std::array<double, 3> flow_acc_world{};  // ambient flow acceleration, world frame, m/s^2
  bool valid = false;                      // becomes true after the first successful write

  // --- write-once static geometry (set by AdhereMujocoHardware::configure(), read freely
  //     afterwards without locking: it is populated synchronously before vis_bridge_ is ever
  //     handed to MujocoViewer/the 500Hz physics thread/any ROS executor thread, so there is no
  //     possible concurrent writer -- mirrors the existing AdhereMujocoHardware::pad_yz_
  //     precedent) ---
  std::array<std::array<double, 3>, 4> pad_site_pos_body{};  // [TL,TR,BL,BR] body-local (x,y,z)

  // --- GUI -> hardware control requests (lock-free atomics) ---
  std::atomic<bool> paused{false};
  std::atomic<bool> reset_request{false};
  std::atomic<bool> step_once_request{false};  // single-tick advance while paused (debugging)
  std::atomic<bool> reset_stats_request{false};  // clears the cumulative liftoff/kern counters
  std::atomic<int> wave_mode_cmd{0};  // 0 = no request pending; 1/2 = switch request from GUI
  std::array<std::atomic<double>, 8> manual_ctrl{};  // additive manual slider thrust, N
  std::array<std::atomic<double>, 6> xfrc_perturb{};  // [Fx,Fy,Fz,Tx,Ty,Tz] on rov_base, world
  std::atomic<int> hydro_mode{0};  // mirrors HydroModelMode: 0=ParametricLumped, 1=StonefishGeometryDriven
  std::atomic<double> sim_speed_factor{1.0};  // <=1.0, slow-motion scale for the physics thread period
  std::atomic<bool> enable_thruster_lag{false};  // first-order ESC lag (tau_T = 0.015s), off by default

  // --- hardware -> GUI scalar telemetry (lock-free atomics) ---
  std::atomic<double> actual_physics_hz{0.0};
  std::atomic<double> step_work_ms{0.0};  // EMA of read()+update()+write() pure compute time
  std::atomic<double> sim_time{0.0};
  std::atomic<int> wave_mode_active{1};
  std::atomic<double> wave_tau{0.0};
  std::atomic<int> hydro_patch_count{0};   // HydroGeometryModel::patches.size()
  std::atomic<double> kern_val{0.0};       // Rhombic Kern K_val
  std::atomic<double> m_kern{0.0};         // Rhombic Kern margin, 1 - K_val
  // True 500Hz cumulative telemetry (Phase 2.11 Section 2): every physics step (not just the
  // ~60Hz GUI-snapshot cadence) is classified and accumulated, so transient <5ms contact events
  // are never aliased away. liftoff_ratio/corner_trap_ratio are fractions in [0,1]; multiply by
  // 100 for the CLAUDE.md Sec.4 "Cumulative Liftoff/Corner-Trap Time Ratio (%)" benchmark figures.
  std::atomic<double> liftoff_ratio{0.0};       // cumulative liftoff time / total eval time
  std::atomic<double> corner_trap_ratio{0.0};   // cumulative corner-trap time / total eval time
  std::atomic<long long> liftoff_events{0};     // discrete liftoff event count (rising edges)
  std::atomic<int> active_pads{4};              // 4 - (pads currently at/below F_N <= 1e-3 N)
  std::atomic<double> cop_y{0.0};          // Center of Pressure, body-local y, m
  std::atomic<double> cop_z{0.0};          // Center of Pressure, body-local z, m
};

}  // namespace adhere_mujoco_hardware

#endif  // ADHERE_MUJOCO_HARDWARE__MUJOCO_VIS_BRIDGE_HPP_
