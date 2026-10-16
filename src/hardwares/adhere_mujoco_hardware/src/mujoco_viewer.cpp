#include "adhere_mujoco_hardware/mujoco_viewer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <utility>
#include <vector>

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
constexpr int kUidScreenshot = 104;
constexpr int kUidCamWallYZ = 105;
constexpr int kUidCamTopXY = 106;
constexpr int kUidCamSideXZ = 107;
constexpr int kUidResetStats = 108;

// Section C4: slow-motion-only speed scale, indexed by the "Sim Speed" radio.
constexpr double kSimSpeedValues[4] = {1.0, 0.5, 0.2, 0.1};

// Appends a single mjITEM_CHECKBYTE item directly bound to *pdata (used throughout buildUi0()
// to bind MuJoCo-native toggles straight to a field inside opt_/scn_, e.g. &opt_.flags[idx] or
// &scn_.flags[idx], instead of introducing a dedicated mirror member per toggle).
void addCheckbox(mjUI * ui, int section, const char * name, mjtByte * pdata)
{
  mjuiDef def[2] = {};
  def[0].type = mjITEM_CHECKBYTE;
  std::snprintf(def[0].name, sizeof(def[0].name), "%s", name);
  def[0].state = 1;
  def[0].pdata = pdata;
  def[1].type = mjITEM_END;
  mjui_addToSection(ui, section, def);
}
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

  // Reassert our own tracked scroll position (see header comment on ui0_scroll_/ui1_scroll_):
  // mjui_resize() does not reliably keep ui0_/ui1_.scroll pinned to where the user left it
  // once content is taller than the panel.
  ui0_.scroll = std::clamp(ui0_scroll_, 0, std::max(0, ui0_.maxheight - ui0_.height));
  ui1_.scroll = std::clamp(ui1_scroll_, 0, std::max(0, ui1_.maxheight - ui1_.height));

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
  ui0_.nsect = 0;

  mjuiDef def_sim[] = {
    {mjITEM_SECTION, "Simulation", mjSECT_OPEN, nullptr, "", 0},
    {mjITEM_CHECKINT, "Running (Space)", 1, &ui_running_, "", 0},
    {mjITEM_BUTTON, "Step Once (S)", 1, nullptr, "", 0},
    {mjITEM_BUTTON, "Reset (Backspace)", 1, nullptr, "", 0},
    {mjITEM_BUTTON, "Reset Stats", 1, nullptr, "", 0},
    {mjITEM_RADIO, "Sim Speed", 4, &ui_sim_speed_, "1x\n0.5x\n0.2x\n0.1x", 0},
    {mjITEM_BUTTON, "Default Orbit", 1, nullptr, "", 0},
    {mjITEM_BUTTON, "Wall YZ View", 1, nullptr, "", 0},
    {mjITEM_BUTTON, "Top XY View", 1, nullptr, "", 0},
    {mjITEM_BUTTON, "Side XZ View", 1, nullptr, "", 0},
    {mjITEM_CHECKBYTE, "Camera Follow", 1, &ui_cam_follow_, "", 0},
    {mjITEM_BUTTON, "Screenshot", 1, nullptr, "", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}};
  def_sim[2].otherint = kUidStepOnce;
  def_sim[3].otherint = kUidReset;
  def_sim[4].otherint = kUidResetStats;
  def_sim[6].otherint = kUidResetView;
  def_sim[7].otherint = kUidCamWallYZ;
  def_sim[8].otherint = kUidCamTopXY;
  def_sim[9].otherint = kUidCamSideXZ;
  def_sim[11].otherint = kUidScreenshot;
  mjui_add(&ui0_, def_sim);

  mjuiDef def_wave[] = {
    {mjITEM_SECTION, "Wave & Hydro Model", mjSECT_OPEN, nullptr, "", 0},
    {mjITEM_RADIO, "Mode (1/2)", 2, &ui_wave_mode_, "Calm Water\nStation-Keeping Gust", 0},
    {mjITEM_RADIO, "Hydro Model", 2, &ui_hydro_mode_, "Lumped\nStonefish-Mesh", 0},
    {mjITEM_CHECKBYTE, "Kern Diamond", 1, &ui_show_kern_diamond_, "", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}};
  mjui_add(&ui0_, def_wave);

  // Custom ROV-specific overlays (not MuJoCo-native: no corresponding opt_/scn_ flag).
  mjuiDef def_rov[] = {
    {mjITEM_SECTION, "ROV Overlays", mjSECT_OPEN, nullptr, "", 0},
    {mjITEM_CHECKBYTE, "Pad Contact Forces", 1, &ui_show_pad_forces_, "", 0},
    {mjITEM_CHECKBYTE, "Flow Velocity Arrow", 1, &ui_show_flow_arrow_, "", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}};
  mjui_add(&ui0_, def_rov);

  // Core MuJoCo mjvOption visualization toggles every official `simulate` build exposes,
  // bound directly to opt_.flags[...]/opt_.frame (see header comment). "Transparent Hull"
  // directly answers the explicit request to make the ROV body see-through. The remaining
  // official toggles (render flags, geom/site groups, Label) live in buildUi1() -- splitting
  // across both panels keeps either side from needing to scroll past a full screen of content.
  mjuiDef def_vis[] = {
    {mjITEM_SECTION, "Visualize", mjSECT_CLOSED, nullptr, "", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}};
  mjui_add(&ui0_, def_vis);
  const int sect_vis = ui0_.nsect - 1;
  static const std::pair<const char *, int> kVisFlags[] = {
    {"Transparent Hull", mjVIS_TRANSPARENT},
    {"Joint", mjVIS_JOINT},
    {"Actuator", mjVIS_ACTUATOR},
    {"Activation", mjVIS_ACTIVATION},
    {"Camera", mjVIS_CAMERA},
    {"Light", mjVIS_LIGHT},
    {"Constraint", mjVIS_CONSTRAINT},
    {"Contact Point", mjVIS_CONTACTPOINT},
    {"Contact Force", mjVIS_CONTACTFORCE},
    {"Contact Split", mjVIS_CONTACTSPLIT},
    {"Perturb Force", mjVIS_PERTFORCE},
    {"Perturb Object", mjVIS_PERTOBJ},
    {"Center of Mass", mjVIS_COM},
    {"Select Point", mjVIS_SELECT},
    {"Static Body", mjVIS_STATIC},
    {"Inertia Box", mjVIS_INERTIA},
    {"Scale Inertia w/ Mass", mjVIS_SCLINERTIA},
    {"Auto Connect", mjVIS_AUTOCONNECT},
  };
  for (const auto & [name, flag] : kVisFlags) {
    addCheckbox(&ui0_, sect_vis, name, &opt_.flags[flag]);
  }
  {
    mjuiDef def_frame[] = {
      {mjITEM_RADIO, "Coord Frames", 8, &opt_.frame,
       "None\nBody\nGeom\nSite\nCamera\nLight\nContact\nWorld", 0},
      {mjITEM_END, "", 0, nullptr, "", 0}};
    mjui_addToSection(&ui0_, sect_vis, def_frame);
  }
}

void MujocoViewer::buildUi1()
{
  ui1_.nsect = 0;

  mjuiDef def_thrust_header[] = {
    {mjITEM_SECTION, "Thrusters (Manual)", mjSECT_OPEN, nullptr, "", 0},
    {mjITEM_BUTTON, "Zero All", 1, nullptr, "", 0},
    {mjITEM_CHECKBYTE, "Thruster Lag (15ms)", 1, &ui_thruster_lag_, "", 0},
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

  mjuiDef def_scope[] = {
    {mjITEM_SECTION, "Live Scope", mjSECT_OPEN, nullptr, "", 0},
    {mjITEM_RADIO, "Scope Plot", 5, &scope_mode_,
     "Off\nPad Forces F_N\nKern Margin & minF_N\nBody Vel vs Flow\nThruster Commands", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}};
  mjui_add(&ui1_, def_scope);

  // Scene-level render flags (mjtRndFlag), bound directly to scn_.flags[...] -- paired here
  // with Label/geom-site-group visibility (also MuJoCo-native) rather than in buildUi0(), to
  // keep either panel's total content within roughly one screen's height. Collapsed by
  // default (CLAUDE.md Phase 2.10 Section C5): these are rarely-touched MuJoCo-native toggle
  // dumps, expandable on click.
  mjuiDef def_rndflags[] = {
    {mjITEM_SECTION, "Render Flags", mjSECT_CLOSED, nullptr, "", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}};
  mjui_add(&ui1_, def_rndflags);
  const int sect_rnd = ui1_.nsect - 1;
  static const std::pair<const char *, int> kRndFlags[] = {
    {"Wireframe", mjRND_WIREFRAME},
    {"Shadow", mjRND_SHADOW},
    {"Reflection", mjRND_REFLECTION},
    {"Skybox", mjRND_SKYBOX},
    {"Fog", mjRND_FOG},
    {"Haze", mjRND_HAZE},
  };
  for (const auto & [name, flag] : kRndFlags) {
    addCheckbox(&ui1_, sect_rnd, name, &scn_.flags[flag]);
  }

  mjuiDef def_label_sect[] = {
    {mjITEM_SECTION, "Label", mjSECT_CLOSED, nullptr, "", 0},
    {mjITEM_RADIO, "Mode", 17, &opt_.label,
     "None\nBody\nJoint\nGeom\nSite\nCamera\nLight\nTendon\nActuator\nConstraint\nFlex\n"
     "Skin\nSelection\nSelPoint\nContactPt\nContactForce\nIsland",
     0},
    {mjITEM_END, "", 0, nullptr, "", 0}};
  mjui_add(&ui1_, def_label_sect);

  // Geom/site visibility by group, bound directly to opt_.geomgroup[i]/sitegroup[i].
  // rov_wall.xml puts every geom/site in the default group (0), so only group 0 has any
  // visible effect today -- trimmed to just that (instead of all mjNGROUP=6 slots) both
  // because groups 1-5 are inert until the MJCF assigns something to them, and because this
  // panel's total content must stay within roughly one screen's height (see the
  // ui0_scroll_/ui1_scroll_ comment in the header: mjui_resize()'s layout math gets unreliable
  // well past that, independent of our scroll override).
  mjuiDef def_groups[] = {
    {mjITEM_SECTION, "Visibility Groups", mjSECT_CLOSED, nullptr, "", 0},
    {mjITEM_CHECKBYTE, "Geom Group 0", 1, &opt_.geomgroup[0], "", 0},
    {mjITEM_CHECKBYTE, "Site Group 0", 1, &opt_.sitegroup[0], "", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}};
  mjui_add(&ui1_, def_groups);

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

  // Section C2: live scope oscilloscope, initialized once here and populated/rendered every
  // frame from updateAndDrawScope().
  mjv_defaultFigure(&scope_fig_);
  scope_fig_.flg_legend = 1;
  std::snprintf(scope_fig_.title, sizeof(scope_fig_.title), "Live Scope");
  static const float kLineColors[kScopeMaxSeries][3] = {
    {1.0f, 0.2f, 0.2f}, {0.2f, 1.0f, 0.2f}, {0.3f, 0.5f, 1.0f}, {1.0f, 1.0f, 0.2f},
    {1.0f, 0.3f, 1.0f}, {0.2f, 1.0f, 1.0f}, {1.0f, 0.6f, 0.1f}, {0.6f, 0.6f, 1.0f}};
  for (int i = 0; i < kScopeMaxSeries; ++i) {
    scope_fig_.linergb[i][0] = kLineColors[i][0];
    scope_fig_.linergb[i][1] = kLineColors[i][1];
    scope_fig_.linergb[i][2] = kLineColors[i][2];
  }

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
      // Ctrl+Left = torque (rotate perturb), Ctrl+Right = force (translate perturb). Default
      // to the Vertical variants (mjMOUSE_MOVE_V/mjMOUSE_ROTATE_V) so users can drag/pan the
      // ROV along World Z on the wall; holding Shift switches to the Horizontal variants.
      const int move_action = uistate_.shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
      const int rotate_action = uistate_.shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
      if (uistate_.left) {
        mjv_movePerturb(model_, d_vis_, rotate_action, reldx, reldy, &scn_, &pert_);
      } else if (uistate_.right) {
        mjv_movePerturb(model_, d_vis_, move_action, reldx, reldy, &scn_, &pert_);
      }
    } else if (uistate_.left) {
      // Vertical orbit by default (drag up/down tilts elevation); Shift switches to horizontal
      // orbit (drag left/right swings azimuth) -- Phase 2.11 Section 5.
      mjv_moveCamera(model_, uistate_.shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V, reldx, reldy, &cam_);
    } else if (uistate_.right) {
      // Vertical pan by default (drag up/down pans along the wall's Z axis); Shift switches to
      // horizontal pan -- without this, there was no way to pan vertically along the wall.
      mjv_moveCamera(model_, uistate_.shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V, reldx, reldy, &cam_);
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
      // Bug 1 fix: re-seed pert_.refpos/refselpos from the ROV's CURRENT pose every time this
      // branch fires, not just once after the last double-click-select. Without this, dragging
      // Ctrl+Click after the ROV has moved yanks it using a stale reference pose.
      mjv_initPerturb(model_, d_vis_, &scn_, &pert_);
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
    ui0_scroll_ = ui0_.scroll;  // persisted across layoutRects()'s per-frame mjui_resize() calls
  } else if (uistate_.mouserect == 2) {
    mjui_event(&ui1_, &uistate_, &con_);
    ui1_scroll_ = ui1_.scroll;
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
    case kUidResetStats:
      bridge_->reset_stats_request.store(true);
      break;
    case kUidStepOnce:
      stepOnce();
      break;
    case kUidResetView:
      resetCameraView();
      break;
    case kUidCamWallYZ:
      setCameraWallYZ();
      break;
    case kUidCamTopXY:
      setCameraTopXY();
      break;
    case kUidCamSideXZ:
      setCameraSideXZ();
      break;
    case kUidZeroThrusters:
      for (auto & v : thruster_slider_) {
        v = 0.0;
      }
      break;
    case kUidScreenshot:
      saveScreenshot();
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

// Section C3 camera presets. lookat/distance are the convention-independent, load-bearing
// values; azimuth/elevation signs match resetCameraView()'s confirmed-working convention and
// should be visually re-checked in a real desktop session (adjust by +-90/180 deg if MuJoCo's
// azimuth/elevation convention differs from what's assumed here).
void MujocoViewer::setCameraWallYZ()
{
  cam_.distance = 1.5;
  cam_.azimuth = 180.0;
  cam_.elevation = 0.0;
  cam_.lookat[0] = 0.0;
  cam_.lookat[1] = 0.0;
  cam_.lookat[2] = 0.5;
}

void MujocoViewer::setCameraTopXY()
{
  cam_.distance = 2.0;
  cam_.azimuth = -90.0;
  cam_.elevation = -90.0;
  cam_.lookat[0] = 0.0;
  cam_.lookat[1] = 0.0;
  cam_.lookat[2] = 0.5;
}

void MujocoViewer::setCameraSideXZ()
{
  cam_.distance = 2.0;
  cam_.azimuth = -90.0;
  cam_.elevation = 0.0;
  cam_.lookat[0] = 0.0;
  cam_.lookat[1] = 0.0;
  cam_.lookat[2] = 0.5;
}

void MujocoViewer::drawKernDiamond()
{
  if (!kern_geom_cached_ || body_id_ < 0 || kern_Ly_ <= 1e-9 || kern_Lz_ <= 1e-9) {
    return;
  }
  const mjtNum local_pts[4][3] = {
    {kern_x_plane_, kern_Ly_, 0.0}, {kern_x_plane_, 0.0, kern_Lz_},
    {kern_x_plane_, -kern_Ly_, 0.0}, {kern_x_plane_, 0.0, -kern_Lz_}};
  // Outer rigid Support Rectangle (+-L_y x +-L_z), the physical corner-pad footprint the
  // Rhombic Kern diamond is inscribed within -- visualizes the full Corner-Triangle Trap
  // region outside the diamond but inside the rectangle.
  const mjtNum local_rect[4][3] = {
    {kern_x_plane_, kern_Ly_, kern_Lz_}, {kern_x_plane_, -kern_Ly_, kern_Lz_},
    {kern_x_plane_, -kern_Ly_, -kern_Lz_}, {kern_x_plane_, kern_Ly_, -kern_Lz_}};
  const mjtNum * body_pos = d_vis_->xpos + 3 * body_id_;
  const mjtNum * body_quat = d_vis_->xquat + 4 * body_id_;
  mjtNum world_pts[4][3];
  mjtNum world_rect[4][3];
  for (int i = 0; i < 4; ++i) {
    mjtNum rotated[3];
    mju_rotVecQuat(rotated, local_pts[i], body_quat);
    world_pts[i][0] = body_pos[0] + rotated[0];
    world_pts[i][1] = body_pos[1] + rotated[1];
    world_pts[i][2] = body_pos[2] + rotated[2];

    mjtNum rotated_rect[3];
    mju_rotVecQuat(rotated_rect, local_rect[i], body_quat);
    world_rect[i][0] = body_pos[0] + rotated_rect[0];
    world_rect[i][1] = body_pos[1] + rotated_rect[1];
    world_rect[i][2] = body_pos[2] + rotated_rect[2];
  }

  // Phase 2.11 Section 5: the diamond boundary itself is always a fixed Cyan, distinct from the
  // CoP sphere, whose color encodes the live admissibility state -- green (inside the kern, all
  // 4 pads in contact), yellow (Corner-Triangle Trap: K_val > 1.0 but all 4 pads still bearing
  // load), red (any pad actually lifted off).
  const float rect_rgba[4] = {0.6f, 0.6f, 0.6f, 0.5f};
  const float diamond_rgba[4] = {0.1f, 0.85f, 1.0f, 0.75f};
  float cop_rgba[4] = {0.1f, 1.0f, 0.1f, 0.85f};
  if (cached_active_pads_ < 4) {
    cop_rgba[0] = 1.0f;
    cop_rgba[1] = 0.1f;
    cop_rgba[2] = 0.1f;
    cop_rgba[3] = 0.9f;
  } else if (cached_kern_val_ > 1.0) {
    cop_rgba[0] = 1.0f;
    cop_rgba[1] = 0.9f;
    cop_rgba[2] = 0.1f;
    cop_rgba[3] = 0.85f;
  }

  // Grey outer Support Rectangle edges, drawn first so the diamond's own edges render on top.
  for (int i = 0; i < 4 && scn_.ngeom < scn_.maxgeom; ++i) {
    mjvGeom * g = &scn_.geoms[scn_.ngeom];
    mjv_initGeom(g, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, rect_rgba);
    mjv_connector(g, mjGEOM_CAPSULE, 0.003, world_rect[i], world_rect[(i + 1) % 4]);
    scn_.ngeom++;
  }

  for (int i = 0; i < 4 && scn_.ngeom < scn_.maxgeom; ++i) {
    mjvGeom * g = &scn_.geoms[scn_.ngeom];
    mjv_initGeom(g, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, diamond_rgba);
    mjv_connector(g, mjGEOM_CAPSULE, 0.003, world_pts[i], world_pts[(i + 1) % 4]);
    scn_.ngeom++;
  }

  // CoP is physically undefined while fully detached from the wall (sum F_N,i ~ 0) -- do not
  // draw a false CoP sphere at the diamond's center in that case.
  const double total_pad_force =
    cached_pad_force_[0] + cached_pad_force_[1] + cached_pad_force_[2] + cached_pad_force_[3];
  if (total_pad_force > 1e-3 && scn_.ngeom < scn_.maxgeom) {
    const mjtNum local_cop[3] = {kern_x_plane_ + 0.005, cached_cop_y_, cached_cop_z_};
    mjtNum rotated[3];
    mju_rotVecQuat(rotated, local_cop, body_quat);
    const mjtNum cop_world[3] = {
      body_pos[0] + rotated[0], body_pos[1] + rotated[1], body_pos[2] + rotated[2]};
    const mjtNum size[3] = {0.018, 0.018, 0.018};
    mjvGeom * g = &scn_.geoms[scn_.ngeom];
    mjv_initGeom(g, mjGEOM_SPHERE, size, cop_world, nullptr, cop_rgba);
    scn_.ngeom++;
  }
}

void MujocoViewer::updateAndDrawScope(mjrRect full_viewport)
{
  if (scope_mode_ != last_scope_channel_) {
    // Switching channels: drop the old history so stale large-scale values (e.g. 40N
    // thrusters) can never crush a new, smaller-scale signal's auto-ranged Y axis.
    last_scope_channel_ = scope_mode_;
    scope_history_count_ = 0;
    for (auto & series : scope_history_) {
      series.fill(0.0f);
    }
  }

  if (scope_mode_ == 0) {
    return;
  }

  int n_series = 0;
  float samples[kScopeMaxSeries] = {0};
  const char * names[kScopeMaxSeries] = {nullptr};
  switch (scope_mode_) {
    case 1:  // Pad Forces F_N (N)
      n_series = 4;
      for (int i = 0; i < 4; ++i) {
        samples[i] = static_cast<float>(cached_pad_force_[i]);
      }
      names[0] = "TL";
      names[1] = "TR";
      names[2] = "BL";
      names[3] = "BR";
      break;
    case 2: {  // Kern Margin M_kern & min F_N
      n_series = 2;
      samples[0] = static_cast<float>(cached_m_kern_);
      const double min_f = *std::min_element(cached_pad_force_.begin(), cached_pad_force_.end());
      samples[1] = static_cast<float>(min_f / 10.0);
      names[0] = "M_kern";
      names[1] = "minF_N/10";
      break;
    }
    case 3: {  // Body Vel vs Flow (m/s)
      n_series = 4;
      double vx_b = 0.0, vy_b = 0.0, vz_b = 0.0;
      if (model_->nq >= 7) {
        const mjtNum quat[4] = {
          d_vis_->qpos[3], d_vis_->qpos[4], d_vis_->qpos[5], d_vis_->qpos[6]};
        mjtNum mat[9];
        mju_quat2Mat(mat, quat);
        const double vx_w = d_vis_->qvel[0], vy_w = d_vis_->qvel[1], vz_w = d_vis_->qvel[2];
        vx_b = mat[0] * vx_w + mat[3] * vy_w + mat[6] * vz_w;
        vy_b = mat[1] * vx_w + mat[4] * vy_w + mat[7] * vz_w;
        vz_b = mat[2] * vx_w + mat[5] * vy_w + mat[8] * vz_w;
      }
      samples[0] = static_cast<float>(vx_b);
      samples[1] = static_cast<float>(vy_b);
      samples[2] = static_cast<float>(vz_b);
      samples[3] = static_cast<float>(cached_flow_vel_world_[1]);
      names[0] = "vx_b";
      names[1] = "vy_b";
      names[2] = "vz_b";
      names[3] = "Vflow_y";
      break;
    }
    case 4:  // Thruster Commands u_i (N)
      n_series = 8;
      for (int i = 0; i < 8 && i < model_->nu; ++i) {
        samples[i] = static_cast<float>(d_vis_->ctrl[i]);
        names[i] = kThrusterLabels[i];
      }
      break;
    default:
      return;
  }

  for (int s = 0; s < n_series; ++s) {
    if (scope_history_count_ >= kScopeHistoryLen) {
      std::copy(
        scope_history_[s].begin() + 1, scope_history_[s].end(), scope_history_[s].begin());
      scope_history_[s][kScopeHistoryLen - 1] = samples[s];
    } else {
      scope_history_[s][scope_history_count_] = samples[s];
    }
    if (names[s]) {
      std::snprintf(
        scope_fig_.linename[s], sizeof(scope_fig_.linename[s]), "%s", names[s]);
    }
  }
  scope_history_count_ = std::min(scope_history_count_ + 1, kScopeHistoryLen);

  for (int s = 0; s < n_series; ++s) {
    scope_fig_.linepnt[s] = scope_history_count_;
    for (int k = 0; k < scope_history_count_; ++k) {
      scope_fig_.linedata[s][2 * k] = static_cast<float>(k);
      scope_fig_.linedata[s][2 * k + 1] = scope_history_[s][k];
    }
  }
  for (int s = n_series; s < kScopeMaxSeries; ++s) {
    scope_fig_.linepnt[s] = 0;
  }

  mjrRect scope_vp;
  scope_vp.width = 420;
  scope_vp.height = 220;
  scope_vp.left = std::max(0, full_viewport.width - scope_vp.width - 10);
  scope_vp.bottom = 10;
  mjr_figure(scope_vp, &scope_fig_, &con_);
}

void MujocoViewer::saveScreenshot()
{
  // Official MuJoCo `simulate` saves PNG; writing a simple PPM here instead avoids pulling in
  // a PNG encoder dependency. PPM (P6) is still natively readable by GIMP/most image viewers,
  // and trivially convertible (e.g. `convert shot.ppm shot.png`) if a PNG is needed.
  mjrRect viewport = uistate_.rect[0];
  std::vector<unsigned char> rgb(static_cast<size_t>(viewport.width) * viewport.height * 3);
  mjr_readPixels(rgb.data(), nullptr, viewport, &con_);

  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  char path[256];
  std::snprintf(path, sizeof(path), "/tmp/adhere_mujoco_screenshot_%lld.ppm", static_cast<long long>(ms));

  std::ofstream out(path, std::ios::binary);
  if (!out) {
    std::fprintf(stderr, "[MujocoViewer] Failed to open '%s' for screenshot\n", path);
    return;
  }
  out << "P6\n" << viewport.width << " " << viewport.height << "\n255\n";
  // mjr_readPixels returns rows bottom-up (OpenGL convention); PPM expects top-down rows.
  const size_t row_bytes = static_cast<size_t>(viewport.width) * 3;
  for (int row = viewport.height - 1; row >= 0; --row) {
    out.write(reinterpret_cast<const char *>(rgb.data() + static_cast<size_t>(row) * row_bytes), row_bytes);
  }
  std::fprintf(stdout, "[MujocoViewer] Screenshot saved to %s\n", path);
}

void MujocoViewer::drawOverlayTelemetry(mjrRect viewport)
{
  char title[1024];
  char content[1024];

  // Bug 4 fix: no locking here -- reads the single per-frame snapshot cache renderFrame()
  // already populated earlier this same frame.
  const std::array<double, 3> & flow = cached_flow_vel_world_;
  const std::array<double, 4> & pad_force = cached_pad_force_;

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
    // Bug 5 fix: qvel[0:3] is WORLD-frame linear velocity per MuJoCo's freejoint convention
    // (qvel[3:6] is body-frame angular velocity, already correct below) -- rotate by R^T
    // (mat's transpose, already computed above for the Euler-angle display) to get the true
    // body-frame velocity this row is labeled as.
    const double vx_w = d_vis_->qvel[0], vy_w = d_vis_->qvel[1], vz_w = d_vis_->qvel[2];
    vx = mat[0] * vx_w + mat[3] * vy_w + mat[6] * vz_w;
    vy = mat[1] * vx_w + mat[4] * vy_w + mat[7] * vz_w;
    vz = mat[2] * vx_w + mat[5] * vy_w + mat[8] * vz_w;
    wx = d_vis_->qvel[3];
    wy = d_vis_->qvel[4];
    wz = d_vis_->qvel[5];
  }

  const double hz = bridge_->actual_physics_hz.load();
  const double work_ms = bridge_->step_work_ms.load();
  const double sim_t = bridge_->sim_time.load();
  const int wave_mode = bridge_->wave_mode_active.load();
  const double tau = bridge_->wave_tau.load();
  const char * hydro_name = ui_hydro_mode_ == 1 ? "Stonefish-Mesh" : "Lumped";

  std::snprintf(
    title, sizeof(title),
    "Physics Rate\nSim Time\nWave Mode\nFlow Velocity\n"
    "Pad TL / TR / BL / BR\nPose (X Y Z)\nPose (phi theta psi)\nBody Vel (vx vy vz)\n"
    "Body AngVel (wx wy wz)\nHydro Mode\nKern Margin M_kern\nActive Pads\n"
    "Liftoff Ratio (cum.)\nCorner-Trap Ratio (cum.)\nLiftoff Events");
  std::snprintf(
    content, sizeof(content),
    "%.1f Hz (work: %.3f ms)\n%.2f s\n%s (tau=%.2fs)\n[%.2f, %.2f, %.2f] m/s\n"
    "%.1f / %.1f / %.1f / %.1f N\n[%.3f, %.3f, %.3f] m\n[%.1f, %.1f, %.1f] deg\n[%.3f, %.3f, %.3f] m/s\n"
    "[%.3f, %.3f, %.3f] rad/s\n%s (%d patches)\n%.3f (K_val=%.3f)\n%d / 4\n%.1f%%\n%.1f%%\n%lld",
    hz, work_ms, sim_t, wave_mode == 2 ? "Gust" : "Calm", tau, flow[0], flow[1], flow[2],
    pad_force[0], pad_force[1], pad_force[2], pad_force[3], px, py, pz, phi * 57.2958, theta * 57.2958,
    psi * 57.2958, vx, vy, vz, wx, wy, wz, hydro_name, cached_hydro_patch_count_, cached_m_kern_,
    cached_kern_val_, cached_active_pads_, cached_liftoff_ratio_ * 100.0,
    cached_corner_trap_ratio_ * 100.0, cached_liftoff_events_);

  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, title, content, &con_);
}

void MujocoViewer::renderFrame()
{
  // Bug 4 fix: exactly one bridge mutex critical section per frame, populating every cached_*
  // member this frame needs (qpos/qvel/ctrl go straight into d_vis_, which already serves as
  // their own per-frame cache for mj_forward()/rendering/drawOverlayTelemetry()).
  {
    std::unique_lock<std::mutex> lock(bridge_->mtx, std::try_to_lock);
    if (lock.owns_lock() && bridge_->valid) {
      std::memcpy(d_vis_->qpos, bridge_->qpos.data(), bridge_->qpos.size() * sizeof(double));
      std::memcpy(d_vis_->qvel, bridge_->qvel.data(), bridge_->qvel.size() * sizeof(double));
      for (size_t i = 0; i < bridge_->ctrl.size() && i < static_cast<size_t>(model_->nu); ++i) {
        d_vis_->ctrl[i] = bridge_->ctrl[i];
      }
      cached_pad_force_ = bridge_->pad_force;
      cached_pad_v_sep_ = bridge_->pad_v_sep;
      cached_flow_vel_world_ = bridge_->flow_vel_world;
      cached_flow_acc_world_ = bridge_->flow_acc_world;
      cached_valid_ = true;
      if (!kern_geom_cached_) {
        // Write-once pad geometry (CLAUDE.md Phase 2.10 Section A3): cache L_y/L_z/x_plane
        // the first time it's observed, no need to recompute every frame afterwards.
        double ly = 0.0, lz = 0.0;
        for (const auto & pos : bridge_->pad_site_pos_body) {
          ly = std::max(ly, std::abs(pos[1]));
          lz = std::max(lz, std::abs(pos[2]));
        }
        kern_Ly_ = ly;
        kern_Lz_ = lz;
        kern_x_plane_ = bridge_->pad_site_pos_body[0][0];
        kern_geom_cached_ = true;
      }
    }
    cached_kern_val_ = bridge_->kern_val.load();
    cached_m_kern_ = bridge_->m_kern.load();
    cached_liftoff_ratio_ = bridge_->liftoff_ratio.load();
    cached_corner_trap_ratio_ = bridge_->corner_trap_ratio.load();
    cached_liftoff_events_ = bridge_->liftoff_events.load();
    cached_active_pads_ = bridge_->active_pads.load();
    cached_cop_y_ = bridge_->cop_y.load();
    cached_cop_z_ = bridge_->cop_z.load();
    cached_hydro_patch_count_ = bridge_->hydro_patch_count.load();
  }

  // Note: opt_.flags[*], opt_.frame, opt_.label, opt_.geomgroup/sitegroup[*], and
  // scn_.flags[*] are all mjUI-bound directly (see buildUi0()) and therefore already current
  // here -- mjui_event() writes straight into these fields, no per-frame copy needed.

  // Bug 1 fix: clear any stale xfrc_applied on d_vis_ before (re-)applying this frame's
  // perturb force, so a force left over from a previous frame (e.g. the first frame after
  // pert_.active toggles off) never leaks into mj_forward().
  if (body_id_ >= 0) {
    for (int i = 0; i < 6; ++i) {
      d_vis_->xfrc_applied[6 * body_id_ + i] = 0.0;
    }
  }
  if (pert_.active) {
    mjv_applyPerturbForce(model_, d_vis_, &pert_);
    if (pert_select_body_ == body_id_) {
      for (int i = 0; i < 6; ++i) {
        bridge_->xfrc_perturb[i].store(d_vis_->xfrc_applied[6 * body_id_ + i]);
      }
    }
  }

  // Section C3: Camera Follow overwrites only lookat, composing with manual orbit/presets.
  if (ui_cam_follow_ && body_id_ >= 0) {
    cam_.lookat[0] = d_vis_->xipos[3 * body_id_ + 0];
    cam_.lookat[1] = d_vis_->xipos[3 * body_id_ + 1];
    cam_.lookat[2] = d_vis_->xipos[3 * body_id_ + 2];
  }

  mj_forward(model_, d_vis_);

  int fb_w = 0, fb_h = 0;
  glfwGetFramebufferSize(window_, &fb_w, &fb_h);
  layoutRects(fb_w, fb_h);
  mjrRect viewport = uistate_.rect[3];

  mjv_updateScene(model_, d_vis_, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);

  // Pad-force bars: short capsules from each pad site, extending along +X (away from the
  // wall) proportional to the measured normal force. Reads cached_pad_force_ (Bug 4 fix) --
  // no per-block locking.
  if (ui_show_pad_forces_) {
    for (int i = 0; i < kPadCount && scn_.ngeom < scn_.maxgeom; ++i) {
      if (pad_site_id_[i] < 0) {
        continue;
      }
      const double * site_pos = d_vis_->site_xpos + 3 * pad_site_id_[i];
      const double length = std::max(0.0, cached_pad_force_[static_cast<size_t>(i)]) * kForceScale;
      const mjtNum from[3] = {site_pos[0], site_pos[1], site_pos[2]};
      const mjtNum to[3] = {site_pos[0] + length, site_pos[1], site_pos[2]};
      mjvGeom * g = &scn_.geoms[scn_.ngeom];
      const float rgba[4] = {0.1f, 1.0f, 0.1f, 1.0f};
      mjv_initGeom(g, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, rgba);
      mjv_connector(g, mjGEOM_CAPSULE, 0.01, from, to);
      scn_.ngeom++;
    }
  }

  // Ambient flow-velocity arrow from the body CoM. Reads cached_flow_vel_world_ (Bug 4 fix).
  if (ui_show_flow_arrow_ && body_id_ >= 0 && scn_.ngeom < scn_.maxgeom) {
    const double * com = d_vis_->xipos + 3 * body_id_;
    const mjtNum from[3] = {com[0], com[1], com[2]};
    const mjtNum to[3] = {
      com[0] + cached_flow_vel_world_[0] * kFlowScale, com[1] + cached_flow_vel_world_[1] * kFlowScale,
      com[2] + cached_flow_vel_world_[2] * kFlowScale};
    mjvGeom * g = &scn_.geoms[scn_.ngeom];
    const float rgba[4] = {0.2f, 0.6f, 1.0f, 1.0f};
    mjv_initGeom(g, mjGEOM_ARROW, nullptr, nullptr, nullptr, rgba);
    mjv_connector(g, mjGEOM_ARROW, 0.015, from, to);
    scn_.ngeom++;
  }

  // Section C1: Rhombic Kern diamond + live CoP marker.
  if (ui_show_kern_diamond_) {
    drawKernDiamond();
  }

  mjr_render(viewport, &scn_, &con_);
  drawOverlayTelemetry(viewport);
  updateAndDrawScope(uistate_.rect[0]);

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

  // Push GUI-side control state to the hardware thread.
  bridge_->paused.store(ui_running_ == 0);

  // Bug 6 fix: bidirectional wave-mode sync. Read the external bridge state FIRST; if it
  // changed since last frame (e.g. the /rov/wave_mode_cmd ROS topic, or onKey()'s 1/2 keys
  // which already write ui_wave_mode_ directly), pull it into ui_wave_mode_ and mark it as
  // "already synced" so the edit-detect below does not immediately push it right back out.
  const int external_mode = bridge_->wave_mode_active.load();
  if (external_mode != last_external_wave_mode_) {
    last_external_wave_mode_ = external_mode;
    ui_wave_mode_ = external_mode - 1;
    last_ui_wave_mode_ = ui_wave_mode_;
  }
  if (ui_wave_mode_ != last_ui_wave_mode_) {
    last_ui_wave_mode_ = ui_wave_mode_;
    bridge_->wave_mode_cmd.store(ui_wave_mode_ + 1);
  }

  bridge_->hydro_mode.store(ui_hydro_mode_);
  bridge_->sim_speed_factor.store(kSimSpeedValues[ui_sim_speed_]);
  bridge_->enable_thruster_lag.store(ui_thruster_lag_ != 0);

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
