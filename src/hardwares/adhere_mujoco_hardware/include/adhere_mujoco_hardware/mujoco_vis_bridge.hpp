#ifndef ADHERE_MUJOCO_HARDWARE__MUJOCO_VIS_BRIDGE_HPP_
#define ADHERE_MUJOCO_HARDWARE__MUJOCO_VIS_BRIDGE_HPP_

#include <array>
#include <mutex>
#include <vector>

namespace adhere_mujoco_hardware
{

// In-process 500Hz<->60Hz snapshot bridge between AdhereMujocoHardware (producer, writes via
// try_lock() so the 500Hz physics loop never blocks on the renderer) and MujocoViewer
// (consumer, reads once per rendered frame). Sized dynamically to the model's nq/nv at
// construction time since mjModel::nq/nv are not compile-time constants.
struct MujocoVisBridge
{
  explicit MujocoVisBridge(int nq, int nv)
  : qpos(static_cast<size_t>(nq), 0.0), qvel(static_cast<size_t>(nv), 0.0)
  {
  }

  std::mutex mtx;
  std::vector<double> qpos;
  std::vector<double> qvel;
  std::array<double, 8> ctrl{};
  std::array<double, 4> pad_force{};  // [TL, TR, BL, BR], N
  std::array<double, 3> flow_vel_world{};  // ambient flow velocity, world frame, m/s
  bool valid = false;  // becomes true after the first successful write
};

}  // namespace adhere_mujoco_hardware

#endif  // ADHERE_MUJOCO_HARDWARE__MUJOCO_VIS_BRIDGE_HPP_
