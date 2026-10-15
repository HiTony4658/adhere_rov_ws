#ifndef ADHERE_MUJOCO_HARDWARE__MUJOCO_VIEWER_HPP_
#define ADHERE_MUJOCO_HARDWARE__MUJOCO_VIEWER_HPP_

#include <atomic>
#include <memory>
#include <thread>

#include <mujoco/mujoco.h>

#include "adhere_mujoco_hardware/mujoco_vis_bridge.hpp"

namespace adhere_mujoco_hardware
{

// 60Hz GLFW/OpenGL rendering frontend, owning a secondary mjData* (d_vis_) so its
// mj_forward() calls never touch the 500Hz physics thread's authoritative mjData. Runs on
// its own std::thread; synchronizes with AdhereMujocoHardware purely through
// MujocoVisBridge::try_lock(), so a stalled/slow GPU can never block the real-time 500Hz
// write() loop. `headless=true` skips GLFW/window creation entirely (no-op thread).
class MujocoViewer
{
public:
  MujocoViewer(const mjModel * model, std::shared_ptr<MujocoVisBridge> bridge, bool headless);
  ~MujocoViewer();

  MujocoViewer(const MujocoViewer &) = delete;
  MujocoViewer & operator=(const MujocoViewer &) = delete;

  void start();
  void stop();

private:
  void renderLoop();

  const mjModel * model_;
  mjData * d_vis_ = nullptr;
  std::shared_ptr<MujocoVisBridge> bridge_;
  bool headless_;

  std::thread thread_;
  std::atomic<bool> running_{false};
};

}  // namespace adhere_mujoco_hardware

#endif  // ADHERE_MUJOCO_HARDWARE__MUJOCO_VIEWER_HPP_
