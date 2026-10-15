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
  bool valid = false;                      // becomes true after the first successful write

  // --- GUI -> hardware control requests (lock-free atomics) ---
  std::atomic<bool> paused{false};
  std::atomic<bool> reset_request{false};
  std::atomic<bool> step_once_request{false};  // single-tick advance while paused (debugging)
  std::atomic<int> wave_mode_cmd{0};  // 0 = no request pending; 1/2 = switch request from GUI
  std::array<std::atomic<double>, 8> manual_ctrl{};  // additive manual slider thrust, N
  std::array<std::atomic<double>, 6> xfrc_perturb{};  // [Fx,Fy,Fz,Tx,Ty,Tz] on rov_base, world

  // --- hardware -> GUI scalar telemetry (lock-free atomics) ---
  std::atomic<double> actual_physics_hz{0.0};
  std::atomic<double> sim_time{0.0};
  std::atomic<int> wave_mode_active{1};
  std::atomic<double> wave_tau{0.0};
};

}  // namespace adhere_mujoco_hardware

#endif  // ADHERE_MUJOCO_HARDWARE__MUJOCO_VIS_BRIDGE_HPP_
