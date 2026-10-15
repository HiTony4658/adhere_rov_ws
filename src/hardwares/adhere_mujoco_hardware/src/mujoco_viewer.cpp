#include "adhere_mujoco_hardware/mujoco_viewer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>

#include <GLFW/glfw3.h>

namespace adhere_mujoco_hardware
{

namespace
{
// Body id of rov_base and site ids of the 4 pads, resolved lazily inside renderLoop() since
// mj_name2id needs the model pointer (available from construction, so this could also be
// precomputed in the constructor -- kept local to renderLoop for locality with its draw calls).
constexpr int kPadCount = 4;
constexpr const char * kPadSiteNames[kPadCount] = {
  "pad_TL_site", "pad_TR_site", "pad_BL_site", "pad_BR_site"};
}  // namespace

MujocoViewer::MujocoViewer(
  const mjModel * model, std::shared_ptr<MujocoVisBridge> bridge, bool headless)
: model_(model), bridge_(std::move(bridge)), headless_(headless)
{
  d_vis_ = mj_makeData(model_);
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
  GLFWwindow * window =
    glfwCreateWindow(1280, 900, "adhere_mujoco_hardware: MuJoCo passive-drift view", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  mjvCamera cam;
  mjvOption opt;
  mjvScene scn;
  mjrContext con;
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);
  mjv_makeScene(model_, &scn, 2000);
  mjr_makeContext(model_, &con, mjFONTSCALE_150);

  cam.distance = 2.8;
  cam.azimuth = -135.0;
  cam.elevation = -20.0;
  cam.lookat[0] = 0.0;
  cam.lookat[1] = 0.0;
  cam.lookat[2] = 0.6;

  int pad_site_id[kPadCount];
  for (int i = 0; i < kPadCount; ++i) {
    pad_site_id[i] = mj_name2id(model_, mjOBJ_SITE, kPadSiteNames[i]);
  }
  const int body_id = mj_name2id(model_, mjOBJ_BODY, "rov_base");

  const double kForceScale = 0.01;    // m per N, for pad-force bar length
  const double kFlowScale = 1.0;      // m per (m/s), for the flow-velocity arrow

  while (running_.load() && !glfwWindowShouldClose(window)) {
    {
      std::lock_guard<std::mutex> lock(bridge_->mtx);
      if (bridge_->valid) {
        std::memcpy(d_vis_->qpos, bridge_->qpos.data(), bridge_->qpos.size() * sizeof(double));
        std::memcpy(d_vis_->qvel, bridge_->qvel.data(), bridge_->qvel.size() * sizeof(double));
        for (size_t i = 0; i < bridge_->ctrl.size() && i < static_cast<size_t>(model_->nu); ++i) {
          d_vis_->ctrl[i] = bridge_->ctrl[i];
        }
      }
    }
    mj_forward(model_, d_vis_);

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    mjv_updateScene(model_, d_vis_, &opt, nullptr, &cam, mjCAT_ALL, &scn);

    // Pad-force bars: short capsules from each pad site, extending along +X (away from the
    // wall) proportional to the measured normal force.
    std::array<double, 4> pad_force{};
    std::array<double, 3> flow_vel{};
    {
      std::lock_guard<std::mutex> lock(bridge_->mtx);
      pad_force = bridge_->pad_force;
      flow_vel = bridge_->flow_vel_world;
    }
    for (int i = 0; i < kPadCount && scn.ngeom < scn.maxgeom; ++i) {
      if (pad_site_id[i] < 0) {
        continue;
      }
      const double * site_pos = d_vis_->site_xpos + 3 * pad_site_id[i];
      const double length = std::max(0.0, pad_force[static_cast<size_t>(i)]) * kForceScale;
      const mjtNum from[3] = {site_pos[0], site_pos[1], site_pos[2]};
      const mjtNum to[3] = {site_pos[0] + length, site_pos[1], site_pos[2]};
      mjvGeom * g = &scn.geoms[scn.ngeom];
      const float rgba[4] = {0.1f, 1.0f, 0.1f, 1.0f};
      mjv_initGeom(g, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, rgba);
      mjv_connector(g, mjGEOM_CAPSULE, 0.01, from, to);
      scn.ngeom++;
    }

    // Ambient flow-velocity arrow from the body CoM.
    if (body_id >= 0 && scn.ngeom < scn.maxgeom) {
      const double * com = d_vis_->xipos + 3 * body_id;
      const mjtNum from[3] = {com[0], com[1], com[2]};
      const mjtNum to[3] = {
        com[0] + flow_vel[0] * kFlowScale, com[1] + flow_vel[1] * kFlowScale,
        com[2] + flow_vel[2] * kFlowScale};
      mjvGeom * g = &scn.geoms[scn.ngeom];
      const float rgba[4] = {0.2f, 0.6f, 1.0f, 1.0f};
      mjv_initGeom(g, mjGEOM_ARROW, nullptr, nullptr, nullptr, rgba);
      mjv_connector(g, mjGEOM_ARROW, 0.015, from, to);
      scn.ngeom++;
    }

    mjr_render(viewport, &scn, &con);
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  glfwDestroyWindow(window);
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
}

}  // namespace adhere_mujoco_hardware
