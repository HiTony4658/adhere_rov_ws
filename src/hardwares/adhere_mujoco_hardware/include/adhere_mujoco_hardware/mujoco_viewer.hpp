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
  void handleUiAction(int userid);
  void stepOnce();
  void resetCameraView();
  void saveScreenshot();

  // Camera presets (CLAUDE.md Phase 2.10 Section C3) -- lookat/distance are the
  // convention-independent, load-bearing values; azimuth/elevation signs were chosen to match
  // resetCameraView()'s confirmed-working convention and should be visually re-checked in a
  // real desktop session.
  void setCameraWallYZ();
  void setCameraTopXY();
  void setCameraSideXZ();

  // Draws the 3D Rhombic Kern diamond + live Center-of-Pressure marker on the wall plane
  // (Section C1), appending into scn_.geoms the same way the existing pad-force-bar/flow-
  // arrow blocks already do. Call after mjv_updateScene(), before mjr_render().
  void drawKernDiamond();
  // Appends the current frame's sample(s) to scope_history_ for whichever scope_mode_ is
  // selected, then renders the mjvFigure oscilloscope into a fixed bottom-right rect
  // (Section C2). Call after mjr_render()/drawOverlayTelemetry().
  void updateAndDrawScope(mjrRect full_viewport);

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

  // With ~45 items spread across 6 sections, ui0_'s content is much taller than the window;
  // mjui_resize() (called every frame from layoutRects()) does not reliably preserve the
  // mujoco-internal scroll offset across resizes once content exceeds one screen, so we track
  // our own desired scroll position here and reassert it on ui0_/ui1_ right after every
  // mjui_resize() call -- onScroll() updates these from the mjui_event()-computed value
  // whenever the user actually turns the mouse wheel over a panel.
  int ui0_scroll_ = 0;
  int ui1_scroll_ = 0;

  // mjUI-bound storage (mjITEM_* pdata must be plain, directly-addressable members).
  // Most MuJoCo-native visualization toggles below are NOT mirrored into separate members --
  // they bind mjITEM_CHECKBYTE/RADIO directly to the corresponding field inside opt_/scn_
  // (e.g. &opt_.flags[mjVIS_ACTUATOR], &scn_.flags[mjRND_SHADOW], &opt_.frame, &opt_.label,
  // &opt_.geomgroup[i]/sitegroup[i]) since those fields are already plain, stably-addressed
  // members of this class and mjv_defaultOption()/mjv_defaultScene() already initialize them
  // to MuJoCo's own sane defaults before buildUi0() runs. Only genuinely custom (non-MuJoCo)
  // toggles need a dedicated ui_* member here.
  int ui_running_ = 1;         // 1 = running, 0 = paused (mirrors bridge_->paused, inverted)
  int ui_flow_mode_ = 0;       // radio index: 0 = Calm, 1 = Gust
  mjtByte ui_show_pad_forces_ = 1;  // custom green capsule bars, not a MuJoCo opt_/scn_ flag
  mjtByte ui_show_flow_arrow_ = 1;  // custom blue flow arrow, not a MuJoCo opt_/scn_ flag
  double thruster_slider_[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // CLAUDE.md Phase 2.10 additions.
  int ui_hydro_mode_ = 0;             // radio index: 0 = Lumped, 1 = Stonefish-Mesh
  mjtByte ui_show_kern_diamond_ = 1;  // Section C1 toggle
  int ui_sim_speed_ = 0;              // radio index: 0=1x,1=0.5x,2=0.2x,3=0.1x (Section C4)
  mjtByte ui_cam_follow_ = 0;         // Section C3 "Follow ROV" checkbox
  mjtByte ui_thruster_lag_ = 0;       // Phase 2.11 Section 4 "Thruster Lag (15ms)" checkbox
  mjtByte ui_thruster_deadzone_ = 0;  // "Thruster Deadzone (2.0N)" checkbox
  int scope_mode_ = 0;  // radio index: 0=Off,1=Pad Forces,2=Kern Margin,3=Body Vel vs Flow,
                        // 4=Thruster Commands (Section C2)

  int pad_site_id_[4] = {-1, -1, -1, -1};
  int body_id_ = -1;

  int pert_select_body_ = -1;

  // Previous raw (top-down, GLFW-native) cursor position, used to compute motion deltas
  // independent of the bottom-up flip applied to uistate_.y for mj*/mjUI consumers.
  double last_raw_x_ = 0.0;
  double last_raw_y_ = 0.0;

  // --- Bug 4: single critical-section snapshot cache, populated once per frame at the top
  //     of renderFrame() (qpos/qvel/ctrl are copied straight into d_vis_ instead, which
  //     already serves as their own per-frame cache). ---
  std::array<double, 4> cached_pad_force_{};
  std::array<double, 4> cached_pad_v_sep_{};
  std::array<double, 3> cached_flow_vel_world_{};
  std::array<double, 3> cached_flow_acc_world_{};
  bool cached_valid_ = false;
  double cached_kern_val_ = 0.0;
  double cached_m_kern_ = 0.0;
  double cached_liftoff_ratio_ = 0.0;
  double cached_corner_trap_ratio_ = 0.0;
  long long cached_liftoff_events_ = 0;
  int cached_active_pads_ = 4;
  double cached_cop_y_ = 0.0;
  double cached_cop_z_ = 0.0;
  int cached_hydro_patch_count_ = 0;

  // --- Bug 6: bidirectional wave-mode sync state (replaces the old function-local static). ---
  int last_ui_flow_mode_ = -1;
  int last_external_flow_mode_ = -1;

  // --- Section C1: Rhombic Kern diamond geometry, resolved once from bridge_->pad_site_pos_body
  //     the first time it's observed valid (write-once data, never changes afterwards). ---
  bool kern_geom_cached_ = false;
  double kern_Ly_ = 0.0;
  double kern_Lz_ = 0.0;
  double kern_x_plane_ = 0.0;

  // --- Section C2: live scope oscilloscope. ---
  static constexpr int kScopeHistoryLen = 400;
  static constexpr int kScopeMaxSeries = 8;
  mjvFigure scope_fig_{};
  std::array<std::array<float, kScopeHistoryLen>, kScopeMaxSeries> scope_history_{};
  int scope_history_count_ = 0;
  int last_scope_channel_ = -1;

  std::thread thread_;
  std::atomic<bool> running_{false};
};

}  // namespace adhere_mujoco_hardware

#endif  // ADHERE_MUJOCO_HARDWARE__MUJOCO_VIEWER_HPP_
