#ifndef ADHERE_MUJOCO_HARDWARE__MUJOCO_VIEWER_HPP_
#define ADHERE_MUJOCO_HARDWARE__MUJOCO_VIEWER_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <thread>

#include <mujoco/mujoco.h>

#include "adhere_mujoco_hardware/mujoco_vis_bridge.hpp"

// Forward-declare GLFWwindow instead of #include <GLFW/glfw3.h> here: this header is pulled
// in by adhere_mujoco_hardware.hpp (included from the plugin library's public headers), and
// keeping GLFW out of that include chain avoids leaking a GL/window-system dependency into
// every consumer of AdhereMujocoHardware.
struct GLFWwindow;

namespace adhere_mujoco_hardware
{

// 60Hz GLFW/OpenGL interactive rendering frontend (CLAUDE.md Phase 2.6 Stage 2.2), owning a
// secondary mjData* (d_vis_) so its mj_forward() calls never touch the 500Hz physics thread's
// authoritative mjData. Runs on its own std::thread; synchronizes with AdhereMujocoHardware
// purely through MujocoVisBridge, so a stalled/slow GPU can never block the real-time 500Hz
// write() loop. `headless=true` skips GLFW/window creation entirely (no-op thread).
//
// Behaves like the official standalone MuJoCo `simulate` application: full mouse camera
// orbit/pan/zoom, double-click body selection + Ctrl+drag mjvPerturb force/torque injection,
// a left mjUI sidebar (sim/wave/rendering toggles) and a right mjUI sidebar (8 thruster
// sliders + live telemetry overlay).
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

  // One-time GL/UI setup, called on the render thread after glfwCreateWindow().
  void initUi();
  void layoutRects(int fb_width, int fb_height);
  void buildUi0();
  void buildUi1();
  void renderFrame();
  void drawOverlayTelemetry(mjrRect viewport);

  // GLFW callback trampolines (static -> instance via glfwGetWindowUserPointer).
  static void cursorPosCallback(GLFWwindow * window, double x, double y);
  static void mouseButtonCallback(GLFWwindow * window, int button, int act, int mods);
  static void scrollCallback(GLFWwindow * window, double xoffset, double yoffset);
  static void keyCallback(GLFWwindow * window, int key, int scancode, int act, int mods);
  static void framebufferSizeCallback(GLFWwindow * window, int width, int height);

  void onCursorPos(double x, double y);
  void onMouseButton(int button, int act, int mods);
  void onScroll(double xoffset, double yoffset);
  void onKey(int key, int act, int mods);

  const mjModel * model_;
  mjData * d_vis_ = nullptr;
  std::shared_ptr<MujocoVisBridge> bridge_;
  bool headless_;

  GLFWwindow * window_ = nullptr;

  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
  mjvPerturb pert_;
  mjuiState uistate_{};
  mjUI ui0_{};  // left: simulation / wave / rendering toggles
  mjUI ui1_{};  // right: thruster sliders + telemetry

  bool ui0_visible_ = true;
  bool ui1_visible_ = true;

  // Cached aux-FBO dims (mjr_addAux is relatively expensive and must not be called every
  // frame): layoutRects() only re-creates ui0_/ui1_'s aux buffers when these actually change.
  int aux0_w_ = -1, aux0_h_ = -1;
  int aux1_w_ = -1, aux1_h_ = -1;

  // mjUI-bound storage (mjITEM_* pdata must be plain, directly-addressable members).
  int ui_running_ = 1;         // 1 = running, 0 = paused (mirrors bridge_->paused, inverted)
  int ui_wave_mode_ = 0;       // radio index: 0 = Calm, 1 = Gust
  mjtByte ui_show_pad_forces_ = 1;
  mjtByte ui_show_flow_arrow_ = 1;
  mjtByte ui_transparent_ = 0;
  mjtByte ui_wireframe_ = 0;
  mjtByte ui_show_contact_ = 0;
  int ui_coord_frames_ = 0;    // radio index: 0 = None, 1 = Body, 2 = All
  double thruster_slider_[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  int pad_site_id_[4] = {-1, -1, -1, -1};
  int body_id_ = -1;

  int pert_select_body_ = -1;

  // Previous raw (top-down, GLFW-native) cursor position, used to compute motion deltas
  // independent of the bottom-up flip applied to uistate_.y for mj*/mjUI consumers.
  double last_raw_x_ = 0.0;
  double last_raw_y_ = 0.0;

  std::thread thread_;
  std::atomic<bool> running_{false};
};

}  // namespace adhere_mujoco_hardware

#endif  // ADHERE_MUJOCO_HARDWARE__MUJOCO_VIEWER_HPP_
