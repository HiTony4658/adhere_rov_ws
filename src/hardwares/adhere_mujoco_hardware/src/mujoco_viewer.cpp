#include "adhere_mujoco_hardware/mujoco_viewer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>

#include <GLFW/glfw3.h>

namespace adhere_mujoco_hardware
{

namespace
{
constexpr int kPadCount = 4;
constexpr const char * kPadSiteNames[kPadCount] = {
  "pad_TL_site", "pad_TR_site", "pad_BL_site", "pad_BR_site"};
constexpr const char * kThrusterLabels[8] = {
  "thr_N1", "thr_N2", "thr_N3", "thr_N4", "thr_T5", "thr_T6", "thr_T7", "thr_T8"};

constexpr int kSidePanelWidth = 310;
constexpr double kForceScale = 0.01;  // m per N, for pad-force bar length
constexpr double kFlowScale = 1.0;    // m per (m/s), for the flow-velocity arrow

// mjUI userid codes for action buttons (distinguished in the mjui_event result).
constexpr int kUidReset = 100;
constexpr int kUidStepOnce = 101;
constexpr int kUidResetView = 102;
constexpr int kUidZeroThrusters = 103;
}  // namespace

MujocoViewer::MujocoViewer(
  const mjModel * model, std::shared_ptr<MujocoVisBridge> bridge, bool headless)
: model_(model), bridge_(std::move(bridge)), headless_(headless)
{
  d_vis_ = mj_makeData(model_);
  mjv_defaultPerturb(&pert_);
  body_id_ = mj_name2id(model_, mjOBJ_BODY, "rov_base");
  for (int i = 0; i < kPadCount; ++i) {
    pad_site_id_[i] = mj_name2id(model_, mjOBJ_SITE, kPadSiteNames[i]);
  }
}

MujocoViewer::~MujocoViewer()
{
  stop();
  if (d_vis_) {
    mj_deleteData(d_vis_);
  }
}

void MujocoViewer::start()
{
  if (running_.exchange(true)) {
    return;
  }
  thread_ = std::thread(&MujocoViewer::renderLoop, this);
}

void MujocoViewer::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  if (thread_.joinable()) {
    thread_.join();
  }
}

void MujocoViewer::layoutRects(int fb_width, int fb_height)
{
  uistate_.nrect = 4;
  uistate_.rect[0] = {0, 0, fb_width, fb_height};

  const int left_w = ui0_visible_ ? std::min(kSidePanelWidth, fb_width / 3) : 0;
  const int right_w = ui1_visible_ ? std::min(kSidePanelWidth, fb_width / 3) : 0;

  uistate_.rect[1] = {0, 0, left_w, fb_height};
  uistate_.rect[2] = {fb_width - right_w, 0, right_w, fb_height};
  uistate_.rect[3] = {left_w, 0, std::max(1, fb_width - left_w - right_w), fb_height};

  ui0_.width = kSidePanelWidth;
  ui1_.width = kSidePanelWidth;
  mjui_resize(&ui0_, &con_);
  mjui_resize(&ui1_, &con_);

  // mjUI panels render into dedicated aux framebuffers (mjUI::auxid indexes con_.auxFBO[]);
  // mjr_makeContext() does not allocate these on its own -- mjr_addAux() must be called
  // explicitly, and only when the panel's pixel dimensions actually change (it is relatively
  // expensive and must not run every frame).
  if (left_w > 0 && (uistate_.rect[1].width != aux0_w_ || uistate_.rect[1].height != aux0_h_)) {
    aux0_w_ = uistate_.rect[1].width;
    aux0_h_ = uistate_.rect[1].height;
    mjr_addAux(ui0_.auxid, aux0_w_, aux0_h_, 0, &con_);
  }
  if (right_w > 0 && (uistate_.rect[2].width != aux1_w_ || uistate_.rect[2].height != aux1_h_)) {
    aux1_w_ = uistate_.rect[2].width;
    aux1_h_ = uistate_.rect[2].height;
    mjr_addAux(ui1_.auxid, aux1_w_, aux1_h_, 0, &con_);
  }
}

void MujocoViewer::buildUi0()
{
  ui0_.sect[0].state = mjSECT_FIXED;  // placeholder cleared by mjui_add below
  ui0_.nsect = 0;

  mjuiDef def_sim[] = {
    {mjITEM_SECTION, "Simulation", mjSECT_OPEN, nullptr, "", 0},
    {mjITEM_CHECKINT, "Running (Space)", 1, &ui_running_, "", 0},
    {mjITEM_BUTTON, "Step Once (S)", 1, nullptr, "", 0},
    {mjITEM_BUTTON, "Reset (Backspace)", 1, nullptr, "", 0},
    {mjITEM_BUTTON, "Reset Camera View", 1, nullptr, "", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}};
  def_sim[2].otherint = kUidStepOnce;
  def_sim[3].otherint = kUidReset;
  def_sim[4].otherint = kUidResetView;
  mjui_add(&ui0_, def_sim);

  mjuiDef def_wave[] = {
    {mjITEM_SECTION, "Wave Environment", mjSECT_OPEN, nullptr, "", 0},
    {mjITEM_RADIO, "Mode (1/2)", 2, &ui_wave_mode_, "Calm Water\nStation-Keeping Gust", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}};
  mjui_add(&ui0_, def_wave);

  mjuiDef def_render[] = {
    {mjITEM_SECTION, "Rendering", mjSECT_OPEN, nullptr, "", 0},
    {mjITEM_CHECKBYTE, "Pad Contact Forces", 1, &ui_show_pad_forces_, "", 0},
    {mjITEM_CHECKBYTE, "Flow Velocity Arrow", 1, &ui_show_flow_arrow_, "", 0},
    {mjITEM_CHECKBYTE, "Transparent Hull", 1, &ui_transparent_, "", 0},
    {mjITEM_CHECKBYTE, "Wireframe", 1, &ui_wireframe_, "", 0},
    {mjITEM_CHECKBYTE, "Contact Points", 1, &ui_show_contact_, "", 0},
    {mjITEM_CHECKBYTE, "Center of Mass", 1, &ui_show_com_, "", 0},
    {mjITEM_CHECKBYTE, "Inertia Box", 1, &ui_show_inertia_, "", 0},
    {mjITEM_RADIO, "Coord Frames", 3, &ui_coord_frames_, "None\nBody\nSite", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}};
  mjui_add(&ui0_, def_render);
}

void MujocoViewer::buildUi1()
{
  ui1_.nsect = 0;

  mjuiDef def_thrust_header[] = {
    {mjITEM_SECTION, "Thrusters (Manual)", mjSECT_OPEN, nullptr, "", 0},
    {mjITEM_BUTTON, "Zero All", 1, nullptr, "", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}};
  def_thrust_header[1].otherint = kUidZeroThrusters;
  mjui_add(&ui1_, def_thrust_header);
  for (int i = 0; i < 8; ++i) {
    mjuiDef def_slider[2] = {};
    def_slider[0].type = mjITEM_SLIDERNUM;
    std::snprintf(def_slider[0].name, sizeof(def_slider[0].name), "%s", kThrusterLabels[i]);
    def_slider[0].state = 1;
    def_slider[0].pdata = &thruster_slider_[i];
    std::snprintf(def_slider[0].other, sizeof(def_slider[0].other), "-40 40");
    def_slider[1].type = mjITEM_END;
    mjui_addToSection(&ui1_, 0, def_slider);
  }

  mjuiDef def_tele_header[] = {
    {mjITEM_SECTION, "Telemetry", mjSECT_OPEN, nullptr, "", 0},
    {mjITEM_STATIC, "See overlay (top-left)", 1, nullptr, "", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}};
  mjui_add(&ui1_, def_tele_header);
}

void MujocoViewer::initUi()
{
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultScene(&scn_);
  mjr_defaultContext(&con_);
  mjv_makeScene(model_, &scn_, 2000);
  // mjFONTSCALE_100 (not _150): this display has no HiDPI scaling (framebuffer pixels ==
  // window pixels), and mjui_resize()/mjui_render() compute each panel's actual rendered
  // width from con_'s font-scaled theme spacing -- at 150% the panels render far wider than
  // the kSidePanelWidth assigned to ui0_.width/ui1_.width and overflow past the window edge.
  mjr_makeContext(model_, &con_, mjFONTSCALE_100);

  resetCameraView();

  ui0_.userdata = this;
  ui1_.userdata = this;
  ui0_.spacing = mjui_themeSpacing(0);
  ui0_.color = mjui_themeColor(0);
  ui0_.rectid = 1;
  ui0_.auxid = 0;
  ui1_.spacing = mjui_themeSpacing(0);
  ui1_.color = mjui_themeColor(0);
  ui1_.rectid = 2;
  ui1_.auxid = 1;

  buildUi0();
  buildUi1();

  int fb_w = 1280;
  int fb_h = 900;
  glfwGetFramebufferSize(window_, &fb_w, &fb_h);
  layoutRects(fb_w, fb_h);

  glfwSetWindowUserPointer(window_, this);
  glfwSetCursorPosCallback(window_, &MujocoViewer::cursorPosCallback);
  glfwSetMouseButtonCallback(window_, &MujocoViewer::mouseButtonCallback);
  glfwSetScrollCallback(window_, &MujocoViewer::scrollCallback);
  glfwSetKeyCallback(window_, &MujocoViewer::keyCallback);
  glfwSetFramebufferSizeCallback(window_, &MujocoViewer::framebufferSizeCallback);
}

void MujocoViewer::cursorPosCallback(GLFWwindow * window, double x, double y)
{
  auto * self = static_cast<MujocoViewer *>(glfwGetWindowUserPointer(window));
  if (self) {
    self->onCursorPos(x, y);
  }
}
void MujocoViewer::mouseButtonCallback(GLFWwindow * window, int button, int act, int mods)
{
  auto * self = static_cast<MujocoViewer *>(glfwGetWindowUserPointer(window));
  if (self) {
    self->onMouseButton(button, act, mods);
  }
}
void MujocoViewer::scrollCallback(GLFWwindow * window, double xoffset, double yoffset)
{
  auto * self = static_cast<MujocoViewer *>(glfwGetWindowUserPointer(window));
  if (self) {
    self->onScroll(xoffset, yoffset);
  }
}
void MujocoViewer::keyCallback(GLFWwindow * window, int key, int, int act, int mods)
{
  auto * self = static_cast<MujocoViewer *>(glfwGetWindowUserPointer(window));
  if (self) {
    self->onKey(key, act, mods);
  }
}
void MujocoViewer::framebufferSizeCallback(GLFWwindow * window, int width, int height)
{
  auto * self = static_cast<MujocoViewer *>(glfwGetWindowUserPointer(window));
  if (self) {
    self->layoutRects(width, height);
  }
}

void MujocoViewer::onCursorPos(double x, double y)
{
  const int fb_h = std::max(1, uistate_.rect[0].height);

  // GLFW's cursor callback is top-down (y=0 at the window's top edge), but mjrRect/mjuiState
  // use MuJoCo's bottom-up (OpenGL) convention -- mjrRect's own field is literally named
  // "bottom", not "top". Without this flip, mjui_event()'s internal item/section hit-testing
  // silently picks the wrong row (or nothing), which is why clicks inside the panels appeared
  // to do nothing.
  const double dx = x - last_raw_x_;
  const double dy_raw = y - last_raw_y_;
  last_raw_x_ = x;
  last_raw_y_ = y;
  const double y_mj = fb_h - y;

  uistate_.dx = dx;
  uistate_.dy = -dy_raw;
  uistate_.x = x;
  uistate_.y = y_mj;
  uistate_.type = mjEVENT_MOVE;

  // Route to whichever UI panel currently owns the mouse (or none, for the 3D viewport).
  mjuiItem * it = nullptr;
  if (uistate_.mouserect == 1) {
    it = mjui_event(&ui0_, &uistate_, &con_);
  } else if (uistate_.mouserect == 2) {
    it = mjui_event(&ui1_, &uistate_, &con_);
  }
  if (it) {
    handleUiAction(it->userid);
  }

  if (uistate_.mouserect == 0 || uistate_.mouserect == 3) {
    const double reldx = dx / fb_h;
    const double reldy = dy_raw / fb_h;

    if (uistate_.control && pert_.active) {
      const int action = uistate_.left ? mjMOUSE_MOVE_H : (uistate_.right ? mjMOUSE_MOVE_H : 0);
      // Ctrl+Left = torque (rotate perturb), Ctrl+Right = force (translate perturb).
      if (uistate_.left) {
        mjv_movePerturb(model_, d_vis_, mjMOUSE_ROTATE_H, reldx, reldy, &scn_, &pert_);
      } else if (uistate_.right) {
        mjv_movePerturb(model_, d_vis_, mjMOUSE_MOVE_H, reldx, reldy, &scn_, &pert_);
      }
      (void)action;
    } else if (uistate_.left) {
      mjv_moveCamera(model_, mjMOUSE_ROTATE_H, reldx, reldy, &cam_);
    } else if (uistate_.right) {
      mjv_moveCamera(model_, mjMOUSE_MOVE_H, reldx, reldy, &cam_);
    } else if (uistate_.middle) {
      mjv_moveCamera(model_, mjMOUSE_ZOOM, 0.0, reldy, &cam_);
    }
  }
}

void MujocoViewer::onMouseButton(int button, int act, int mods)
{
  uistate_.left = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  uistate_.right = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
  uistate_.middle = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  uistate_.control = (mods & GLFW_MOD_CONTROL) != 0;
  uistate_.shift = (mods & GLFW_MOD_SHIFT) != 0;
  uistate_.alt = (mods & GLFW_MOD_ALT) != 0;

  double x = 0.0, y = 0.0;
  glfwGetCursorPos(window_, &x, &y);
  const int fb_h = std::max(1, uistate_.rect[0].height);
  last_raw_x_ = x;
  last_raw_y_ = y;
  uistate_.x = x;
  uistate_.y = fb_h - y;  // see onCursorPos: mjrRect/mjuiState are bottom-up, GLFW is top-down.

  // Resolve which rectangle (left UI / right UI / viewport) owns this click.
  uistate_.mouserect = 3;
  if (ui0_visible_ && x >= uistate_.rect[1].left &&
      x < uistate_.rect[1].left + uistate_.rect[1].width) {
    uistate_.mouserect = 1;
  } else if (ui1_visible_ && x >= uistate_.rect[2].left &&
             x < uistate_.rect[2].left + uistate_.rect[2].width) {
    uistate_.mouserect = 2;
  }

  if (act == GLFW_PRESS) {
    uistate_.type = mjEVENT_PRESS;
    uistate_.button = (button == GLFW_MOUSE_BUTTON_LEFT) ? mjBUTTON_LEFT
      : (button == GLFW_MOUSE_BUTTON_RIGHT) ? mjBUTTON_RIGHT : mjBUTTON_MIDDLE;
    uistate_.dragbutton = uistate_.button;

    // Manual double-click detection (GLFW has no native double-click event).
    const double now = glfwGetTime();
    const bool doubleclick =
      (now - uistate_.buttontime) < 0.3 && uistate_.button == mjBUTTON_LEFT;
    uistate_.doubleclick = doubleclick ? 1 : 0;
    uistate_.buttontime = now;

    if (uistate_.mouserect == 1) {
      mjuiItem * it = mjui_event(&ui0_, &uistate_, &con_);
      if (it) {
        handleUiAction(it->userid);
      }
    } else if (uistate_.mouserect == 2) {
      mjuiItem * it = mjui_event(&ui1_, &uistate_, &con_);
      if (it) {
        handleUiAction(it->userid);
      }
    } else if (doubleclick && button == GLFW_MOUSE_BUTTON_LEFT) {
      // Double-click body selection + mjvPerturb initialization.
      const mjrRect & vp = uistate_.rect[3];
      const double relx = (x - vp.left) / std::max(1, vp.width);
      // GLFW cursor y is top-origin; mjv_select expects a bottom-origin fraction. vp spans the
      // full framebuffer height (bottom=0), so this is just a vertical flip.
      const double rely = 1.0 - y / std::max(1, uistate_.rect[0].height);
      int geomid = -1, flexid = -1, skinid = -1;
      mjtNum selpnt[3];
      const int sel = mjv_select(
        model_, d_vis_, &opt_, static_cast<mjtNum>(vp.width) / std::max(1, vp.height), relx, rely,
        &scn_, selpnt, &geomid, &flexid, &skinid);
      if (sel >= 0) {
        pert_select_body_ = sel;
        pert_.select = sel;
        pert_.skinselect = -1;
        mju_copy3(pert_.refselpos, selpnt);
        mjv_initPerturb(model_, d_vis_, &scn_, &pert_);
      } else {
        pert_select_body_ = -1;
        pert_.select = -1;
        pert_.active = 0;
      }
    } else if (uistate_.control && pert_.select > 0) {
      pert_.active = (button == GLFW_MOUSE_BUTTON_LEFT) ? mjPERT_ROTATE : mjPERT_TRANSLATE;
    }
  } else {
    uistate_.type = mjEVENT_RELEASE;
    if (uistate_.mouserect == 1) {
      mjui_event(&ui0_, &uistate_, &con_);
    } else if (uistate_.mouserect == 2) {
      mjui_event(&ui1_, &uistate_, &con_);
    }
    if (!uistate_.left && !uistate_.right) {
      pert_.active = 0;
      for (auto & v : bridge_->xfrc_perturb) {
        v.store(0.0);
      }
    }
  }
}

void MujocoViewer::onScroll(double, double yoffset)
{
  uistate_.type = mjEVENT_SCROLL;
  uistate_.sx = 0.0;
  uistate_.sy = yoffset;
  if (uistate_.mouserect == 1) {
    mjui_event(&ui0_, &uistate_, &con_);
  } else if (uistate_.mouserect == 2) {
    mjui_event(&ui1_, &uistate_, &con_);
  } else {
    mjv_moveCamera(model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &cam_);
  }
}

void MujocoViewer::onKey(int key, int act, int mods)
{
  if (act != GLFW_PRESS) {
    return;
  }
  uistate_.control = (mods & GLFW_MOD_CONTROL) != 0;
  switch (key) {
    case GLFW_KEY_SPACE:
      ui_running_ = ui_running_ ? 0 : 1;
      break;
    case GLFW_KEY_BACKSPACE:
      bridge_->reset_request.store(true);
      break;
    case GLFW_KEY_1:
      ui_wave_mode_ = 0;
      break;
    case GLFW_KEY_2:
      ui_wave_mode_ = 1;
      break;
    case GLFW_KEY_TAB:
      ui0_visible_ = !ui0_visible_;
      {
        int fb_w, fb_h;
        glfwGetFramebufferSize(window_, &fb_w, &fb_h);
        layoutRects(fb_w, fb_h);
      }
      break;
    case GLFW_KEY_S:
      stepOnce();
      break;
    default:
      break;
  }
}

void MujocoViewer::handleUiAction(int userid)
{
  switch (userid) {
    case kUidReset:
      bridge_->reset_request.store(true);
      break;
    case kUidStepOnce:
      stepOnce();
      break;
    case kUidResetView:
      resetCameraView();
      break;
    case kUidZeroThrusters:
      for (auto & v : thruster_slider_) {
        v = 0.0;
      }
      break;
    default:
      break;  // not an action button (slider/checkbox/radio changes are read directly via pdata)
  }
}

void MujocoViewer::stepOnce()
{
  // Only meaningful while paused -- AdhereMujocoHardware::write() only honors
  // step_once_request when vis_bridge_->paused is true (CLAUDE.md Phase 2.7 debugging aid).
  ui_running_ = 0;
  bridge_->paused.store(true);
  bridge_->step_once_request.store(true);
}

void MujocoViewer::resetCameraView()
{
  cam_.distance = 2.8;
  cam_.azimuth = -135.0;
  cam_.elevation = -20.0;
  cam_.lookat[0] = 0.0;
  cam_.lookat[1] = 0.0;
  cam_.lookat[2] = 0.6;
}

void MujocoViewer::drawOverlayTelemetry(mjrRect viewport)
{
  char title[1024];
  char content[1024];

  std::array<double, 3> flow{};
  std::array<double, 4> pad_force{};
  bool valid = false;
  {
    std::lock_guard<std::mutex> lock(bridge_->mtx);
    flow = bridge_->flow_vel_world;
    pad_force = bridge_->pad_force;
    valid = bridge_->valid;
  }
  (void)valid;

  double px = 0.0, py = 0.0, pz = 0.0, phi = 0.0, theta = 0.0, psi = 0.0;
  double vx = 0.0, vy = 0.0, vz = 0.0;
  double wx = 0.0, wy = 0.0, wz = 0.0;
  if (model_->nq >= 7) {
    px = d_vis_->qpos[0];
    py = d_vis_->qpos[1];
    pz = d_vis_->qpos[2];
    const mjtNum quat[4] = {d_vis_->qpos[3], d_vis_->qpos[4], d_vis_->qpos[5], d_vis_->qpos[6]};
    mjtNum mat[9];
    mju_quat2Mat(mat, quat);
    // ZYX Euler extraction from rotation matrix (row-major 3x3).
    psi = std::atan2(mat[3], mat[0]);
    theta = std::asin(std::clamp(-mat[6], -1.0, 1.0));
    phi = std::atan2(mat[7], mat[8]);
    vx = d_vis_->qvel[0];
    vy = d_vis_->qvel[1];
    vz = d_vis_->qvel[2];
    // MuJoCo freejoint qvel convention: [0:3] world-frame linear velocity, [3:6] body-frame
    // angular velocity (gyro-equivalent) -- useful on its own for spotting tumbling/instability
    // while debugging, independent of the linear Body Vel row above.
    wx = d_vis_->qvel[3];
    wy = d_vis_->qvel[4];
    wz = d_vis_->qvel[5];
  }

  const double hz = bridge_->actual_physics_hz.load();
  const double sim_t = bridge_->sim_time.load();
  const int wave_mode = bridge_->wave_mode_active.load();
  const double tau = bridge_->wave_tau.load();

  std::snprintf(
    title, sizeof(title),
    "Physics Rate\nSim Time\nWave Mode\nFlow Velocity\n"
    "Pad TL / TR / BL / BR\nPose (X Y Z)\nPose (phi theta psi)\nBody Vel (vx vy vz)\n"
    "Body AngVel (wx wy wz)");
  std::snprintf(
    content, sizeof(content),
    "%.1f Hz\n%.2f s\n%s (tau=%.2fs)\n[%.2f, %.2f, %.2f] m/s\n"
    "%.1f / %.1f / %.1f / %.1f N\n[%.3f, %.3f, %.3f] m\n[%.1f, %.1f, %.1f] deg\n[%.3f, %.3f, %.3f] m/s\n"
    "[%.3f, %.3f, %.3f] rad/s",
    hz, sim_t, wave_mode == 2 ? "Gust" : "Calm", tau, flow[0], flow[1], flow[2], pad_force[0],
    pad_force[1], pad_force[2], pad_force[3], px, py, pz, phi * 57.2958, theta * 57.2958,
    psi * 57.2958, vx, vy, vz, wx, wy, wz);

  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, title, content, &con_);
}

void MujocoViewer::renderFrame()
{
  {
    std::unique_lock<std::mutex> lock(bridge_->mtx, std::try_to_lock);
    if (lock.owns_lock() && bridge_->valid) {
      std::memcpy(d_vis_->qpos, bridge_->qpos.data(), bridge_->qpos.size() * sizeof(double));
      std::memcpy(d_vis_->qvel, bridge_->qvel.data(), bridge_->qvel.size() * sizeof(double));
      for (size_t i = 0; i < bridge_->ctrl.size() && i < static_cast<size_t>(model_->nu); ++i) {
        d_vis_->ctrl[i] = bridge_->ctrl[i];
      }
    }
  }

  opt_.flags[mjVIS_TRANSPARENT] = ui_transparent_;
  opt_.flags[mjVIS_CONTACTPOINT] = ui_show_contact_;
  opt_.flags[mjVIS_CONTACTFORCE] = ui_show_contact_;
  opt_.flags[mjVIS_COM] = ui_show_com_;
  opt_.flags[mjVIS_INERTIA] = ui_show_inertia_;
  opt_.frame = (ui_coord_frames_ == 1) ? mjFRAME_BODY : (ui_coord_frames_ == 2) ? mjFRAME_SITE : mjFRAME_NONE;

  if (pert_.active) {
    mjv_applyPerturbForce(model_, d_vis_, &pert_);
    if (pert_select_body_ == body_id_) {
      for (int i = 0; i < 6; ++i) {
        bridge_->xfrc_perturb[i].store(d_vis_->xfrc_applied[6 * body_id_ + i]);
      }
    }
  }

  mj_forward(model_, d_vis_);
  scn_.flags[mjRND_WIREFRAME] = ui_wireframe_;

  int fb_w = 0, fb_h = 0;
  glfwGetFramebufferSize(window_, &fb_w, &fb_h);
  layoutRects(fb_w, fb_h);
  mjrRect viewport = uistate_.rect[3];

  mjv_updateScene(model_, d_vis_, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);

  // Pad-force bars: short capsules from each pad site, extending along +X (away from the
  // wall) proportional to the measured normal force.
  if (ui_show_pad_forces_) {
    std::array<double, 4> pad_force{};
    {
      std::lock_guard<std::mutex> lock(bridge_->mtx);
      pad_force = bridge_->pad_force;
    }
    for (int i = 0; i < kPadCount && scn_.ngeom < scn_.maxgeom; ++i) {
      if (pad_site_id_[i] < 0) {
        continue;
      }
      const double * site_pos = d_vis_->site_xpos + 3 * pad_site_id_[i];
      const double length = std::max(0.0, pad_force[static_cast<size_t>(i)]) * kForceScale;
      const mjtNum from[3] = {site_pos[0], site_pos[1], site_pos[2]};
      const mjtNum to[3] = {site_pos[0] + length, site_pos[1], site_pos[2]};
      mjvGeom * g = &scn_.geoms[scn_.ngeom];
      const float rgba[4] = {0.1f, 1.0f, 0.1f, 1.0f};
      mjv_initGeom(g, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, rgba);
      mjv_connector(g, mjGEOM_CAPSULE, 0.01, from, to);
      scn_.ngeom++;
    }
  }

  // Ambient flow-velocity arrow from the body CoM.
  if (ui_show_flow_arrow_ && body_id_ >= 0 && scn_.ngeom < scn_.maxgeom) {
    std::array<double, 3> flow_vel{};
    {
      std::lock_guard<std::mutex> lock(bridge_->mtx);
      flow_vel = bridge_->flow_vel_world;
    }
    const double * com = d_vis_->xipos + 3 * body_id_;
    const mjtNum from[3] = {com[0], com[1], com[2]};
    const mjtNum to[3] = {
      com[0] + flow_vel[0] * kFlowScale, com[1] + flow_vel[1] * kFlowScale,
      com[2] + flow_vel[2] * kFlowScale};
    mjvGeom * g = &scn_.geoms[scn_.ngeom];
    const float rgba[4] = {0.2f, 0.6f, 1.0f, 1.0f};
    mjv_initGeom(g, mjGEOM_ARROW, nullptr, nullptr, nullptr, rgba);
    mjv_connector(g, mjGEOM_ARROW, 0.015, from, to);
    scn_.ngeom++;
  }

  mjr_render(viewport, &scn_, &con_);
  drawOverlayTelemetry(viewport);

  if (ui0_visible_) {
    mjui_update(-1, -1, &ui0_, &uistate_, &con_);
    mjui_render(&ui0_, &uistate_, &con_);
  }
  if (ui1_visible_) {
    mjui_update(-1, -1, &ui1_, &uistate_, &con_);
    mjui_render(&ui1_, &uistate_, &con_);
  }

  glfwSwapBuffers(window_);
  glfwPollEvents();

  // Push GUI-side control state to the hardware thread. Edge-detect the wave-mode radio so
  // repeated identical requests never spuriously re-latch the gust clock inside setWaveMode().
  bridge_->paused.store(ui_running_ == 0);
  static int last_ui_wave_mode = -1;
  if (ui_wave_mode_ != last_ui_wave_mode) {
    last_ui_wave_mode = ui_wave_mode_;
    bridge_->wave_mode_cmd.store(ui_wave_mode_ + 1);
  }
  for (int i = 0; i < 8; ++i) {
    bridge_->manual_ctrl[static_cast<size_t>(i)].store(thruster_slider_[i]);
  }
}

void MujocoViewer::renderLoop()
{
  if (headless_) {
    // No window, no GL context -- this frontend simply does not render. The 500Hz hardware
    // loop still produces vis-bridge snapshots; they are just never consumed.
    while (running_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return;
  }

  if (!glfwInit()) {
    return;
  }
  window_ = glfwCreateWindow(
    1280, 900, "adhere_mujoco_hardware: AdhereMujocoHardware Interactive Viewer", nullptr,
    nullptr);
  if (!window_) {
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  initUi();

  while (running_.load() && !glfwWindowShouldClose(window_)) {
    renderFrame();
  }

  mjv_freeScene(&scn_);
  mjr_freeContext(&con_);
  glfwDestroyWindow(window_);
  window_ = nullptr;
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
}

}  // namespace adhere_mujoco_hardware
