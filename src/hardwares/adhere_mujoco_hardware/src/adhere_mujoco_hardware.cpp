#include "adhere_mujoco_hardware/adhere_mujoco_hardware.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/logging.hpp"

namespace adhere_mujoco_hardware
{

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger("adhere_mujoco_hardware"); }

// mjcb_passive is a plain C function pointer (mjfGeneric), so it cannot capture `this`.
// CLAUDE.md's architecture is single-process/single-instance by design (one
// AdhereMujocoHardware hosts the one authoritative mjModel/mjData for the whole node), so a
// single static back-pointer is sufficient and keeps the passive-force injection calling
// mj_step at exactly the same point the Python reference's mjcb_passive callback did --
// required for <1e-5 numerical parity.
std::atomic<AdhereMujocoHardware *> g_active_instance{nullptr};

constexpr const char * kPadSiteNames[4] = {
  "pad_TL_site", "pad_TR_site", "pad_BL_site", "pad_BR_site"};

// Launch-time model-source override (CLAUDE.md Phase 2.12 Section 1), consulted by the next
// configure() call. Function-local statics (lazily constructed on first call, not namespace-
// scope globals) because they must be reachable before any AdhereMujocoHardware instance
// exists -- see the header comment on setModelSourceOverride() -- while avoiding the
// unspecified cross-translation-unit initialization-order hazard of non-POD global variables.
std::string & pkgDescriptionOverride()
{
  static std::string value;
  return value;
}
std::string & mjcfFileOverride()
{
  static std::string value;
  return value;
}
}  // namespace

void AdhereMujocoHardware::setModelSourceOverride(
  const std::string & pkg_description, const std::string & mjcf_file)
{
  pkgDescriptionOverride() = pkg_description;
  mjcfFileOverride() = mjcf_file;
}

AdhereMujocoHardware::~AdhereMujocoHardware()
{
  stop();
  if (g_active_instance.load() == this) {
    g_active_instance.store(nullptr);
    mjcb_passive = nullptr;
  }
  if (model_) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }
}

hardware_interface::return_type AdhereMujocoHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  // Zero-recompile model switching (CLAUDE.md Phase 2.12 Section 1): priority order is (1) the
  // static setModelSourceOverride() set by adhere_mujoco_node from its pkg_description/
  // mjcf_file node parameters, (2) the <hardware><param name="pkg_description"/"mjcf_file">
  // HardwareInfo entries (useful for standalone pluginlib use without adhere_mujoco_node), (3)
  // the original hardcoded default -- preserving byte-identical behavior for every existing
  // launch invocation that passes neither.
  std::string pkg_description = "simplified_v1_description";
  std::string mjcf_file = "mjcf/rov_wall.xml";
  if (info.hardware_parameters.count("pkg_description")) {
    pkg_description = info.hardware_parameters.at("pkg_description");
  }
  if (info.hardware_parameters.count("mjcf_file")) {
    mjcf_file = info.hardware_parameters.at("mjcf_file");
  }
  if (!pkgDescriptionOverride().empty()) {
    pkg_description = pkgDescriptionOverride();
  }
  if (!mjcfFileOverride().empty()) {
    mjcf_file = mjcfFileOverride();
  }

  const std::string pkg_share = ament_index_cpp::get_package_share_directory(pkg_description);
  const std::string mjcf_path = pkg_share + "/" + mjcf_file;

  char error[1000] = "";
  model_ = mj_loadXML(mjcf_path.c_str(), nullptr, error, sizeof(error));
  if (!model_) {
    RCLCPP_ERROR(logger(), "Failed to load MJCF '%s': %s", mjcf_path.c_str(), error);
    return hardware_interface::return_type::ERROR;
  }

  // Visual-only override, applied after loading rather than editing mjcf/rov_wall.xml (which
  // is kept byte-identical to the read-only mujoco_mppi_ws reference for physics parity): the
  // default body/site coordinate-frame gizmo (model_->vis.scale.framelength/framewidth, used
  // whenever MujocoViewer's "Coord Frames" option is Body/Site) is sized relative to the
  // model's auto-computed mean extent and renders many times larger than this ~0.2-0.6m ROV.
  // Shrunk by >50x (MuJoCo defaults are 1.0 / 0.1) so frame gizmos read as thin markers instead
  // of dominating the scene.
  model_->vis.scale.framelength = 0.04;
  model_->vis.scale.framewidth = 0.002;

  // Root-body resolution fallback chain (CLAUDE.md Phase 2.12 Section 2): "rov_base" first
  // (rov_wall.xml and every existing model), then the quadruped_ros2_control-style convention
  // "base_link", and finally -- for a totally unnamed future model -- the first body owning a
  // free joint (mjJNT_FREE), since this hardware is only ever meaningful for a single floating
  // rigid vehicle.
  body_id_ = mj_name2id(model_, mjOBJ_BODY, "rov_base");
  if (body_id_ < 0) {
    body_id_ = mj_name2id(model_, mjOBJ_BODY, "base_link");
  }
  if (body_id_ < 0) {
    for (int b = 1; b < model_->nbody; ++b) {
      if (model_->body_jntnum[b] == 1 &&
          model_->jnt_type[model_->body_jntadr[b]] == mjJNT_FREE) {
        body_id_ = b;
        break;
      }
    }
  }
  if (body_id_ < 0) {
    RCLCPP_ERROR(
      logger(),
      "No root body found in MJCF '%s' (tried 'rov_base', 'base_link', and any free-joint body)",
      mjcf_path.c_str());
    mj_deleteModel(model_);
    model_ = nullptr;
    return hardware_interface::return_type::ERROR;
  }

  // Constructed here (before the pad-resolution loop below) rather than after, so that loop
  // can write pad_site_pos_body directly into vis_bridge_ -- CLAUDE.md Phase 2.10 Section A3:
  // this is the single place the full 3D pad-site positions are resolved, read unlocked
  // afterwards by both MujocoViewer and adhere_mujoco_node (replacing their respective
  // duplicated/hardcoded copies). Safe unlocked: populated synchronously here, before
  // vis_bridge_ is ever handed to any other thread.
  vis_bridge_ = std::make_shared<MujocoVisBridge>(model_->nq, model_->nv);
  vis_bridge_->root_body_id = body_id_;

  for (size_t i = 0; i < kPadNames.size(); ++i) {
    const std::string sensor_name = std::string("touch_") + kPadNames[i];
    const int sensor_id = mj_name2id(model_, mjOBJ_SENSOR, sensor_name.c_str());
    if (sensor_id < 0) {
      RCLCPP_ERROR(logger(), "Sensor '%s' not found in MJCF", sensor_name.c_str());
      mj_deleteModel(model_);
      model_ = nullptr;
      return hardware_interface::return_type::ERROR;
    }
    pad_touch_sensor_adr_[i] = model_->sensor_adr[sensor_id];

    // CLAUDE.md Phase 2.6 Stage 2.1 bug #6: resolve pad body-local (y, z) from the MJCF site
    // definition at runtime instead of hardcoding it, so this hardware plugin never silently
    // drifts out of sync with mjcf/rov_wall.xml's pad_*_site positions.
    const int site_id = mj_name2id(model_, mjOBJ_SITE, kPadSiteNames[i]);
    if (site_id < 0) {
      RCLCPP_ERROR(logger(), "Site '%s' not found in MJCF", kPadSiteNames[i]);
      mj_deleteModel(model_);
      model_ = nullptr;
      return hardware_interface::return_type::ERROR;
    }
    pad_yz_[i] = {model_->site_pos[3 * site_id + 1], model_->site_pos[3 * site_id + 2]};
    vis_bridge_->pad_site_pos_body[i] = {
      model_->site_pos[3 * site_id + 0], model_->site_pos[3 * site_id + 1],
      model_->site_pos[3 * site_id + 2]};
  }

  for (size_t i = 0; i < kThrusterNames.size(); ++i) {
    const int act_id = mj_name2id(model_, mjOBJ_ACTUATOR, kThrusterNames[i]);
    if (act_id < 0) {
      RCLCPP_ERROR(logger(), "Actuator '%s' not found in MJCF", kThrusterNames[i]);
      mj_deleteModel(model_);
      model_ = nullptr;
      return hardware_interface::return_type::ERROR;
    }
    actuator_id_[i] = act_id;

    // Dynamic thruster rail limits (CLAUDE.md Phase 2.12 Section 3): read straight from this
    // actuator's own ctrlrange instead of a hardcoded +-40N constant, so a future model
    // declaring different bounds in its MJCF never needs a C++ recompile. Falls back to
    // [-40, 40] only if the actuator declares no limit (ctrlrange[0] >= ctrlrange[1]).
    const bool has_ctrlrange =
      model_->actuator_ctrllimited[act_id] != 0 &&
      model_->actuator_ctrlrange[2 * act_id + 0] < model_->actuator_ctrlrange[2 * act_id + 1];
    thruster_ctrl_min_[i] = has_ctrlrange ? model_->actuator_ctrlrange[2 * act_id + 0] : -40.0;
    thruster_ctrl_max_[i] = has_ctrlrange ? model_->actuator_ctrlrange[2 * act_id + 1] : 40.0;
    vis_bridge_->thruster_ctrl_min[i] = thruster_ctrl_min_[i];
    vis_bridge_->thruster_ctrl_max[i] = thruster_ctrl_max_[i];
  }

  // Always starts in Calm Water (CLAUDE.md Phase 2.5 Stage 2.3): both plugins are constructed
  // up front and selected at runtime via setWaveMode(), never by a static launch-time param.
  calm_plugin_ = std::make_unique<CalmWaterPlugin>();
  gust_plugin_ = std::make_unique<StationKeepingGustPlugin>();

  // Optional per-model MJCF <custom><numeric> overrides (CLAUDE.md Phase 2.12 Section 4):
  // a no-op for rov_wall.xml (it declares none of the "hydro_*" numeric names), preserving
  // every existing HydroParams default bit-for-bit; a future model's MJCF can override any
  // subset of them without touching this header/source.
  HydroParams::applyMjModelOverrides(model_, calm_plugin_->mutableHydroParams());
  HydroParams::applyMjModelOverrides(model_, gust_plugin_->mutableHydroParams());

  wave_mode_.store(1);
  wave_trigger_time_.store(0.0);
  RCLCPP_INFO(logger(), "AdhereMujocoHardware: configured, starting in Calm Water mode");

  // MujocoViewer is deferred to start(): pure-xacro decoupling (CLAUDE.md Phase 2.6 Stage 2.1)
  // means `headless` is no longer known at configure() time -- it arrives later via
  // setHeadless(), called by adhere_mujoco_node through activeInstance() after this configure()
  // call returns but before resource_manager->start_components() triggers start().

  // CLAUDE.md Phase 2.10 Section B8: tessellate this robot's own geoms into a
  // HydroGeometryModel once, cached for the lifetime of this instance and reused by every
  // kStonefishGeometryDriven compute() call. rho_water is sourced from calm_plugin_'s
  // configured HydroParams (hydrodynamic-accuracy audit fix): water density is an environmental
  // constant every FlowEnvironmentPlugin is expected to agree on, not a per-flow-profile
  // property, so the calm-water baseline is the canonical source of truth for it.
  hydro_geometry_model_ = HydroGeometryModel::buildFromMjModel(
    model_, body_id_, calm_plugin_->getHydroParams().rho_water);
  RCLCPP_INFO(
    logger(), "AdhereMujocoHardware: HydroGeometryModel built with %zu surface patches",
    hydro_geometry_model_.patches.size());

  hw_states_.fill(0.0);
  hw_commands_.fill(0.0);

  // Set before start()/stop() so setHeadless() can reach this instance even though
  // mjcb_passive (which also needs data_ to exist) is only installed in start().
  g_active_instance.store(this);

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> AdhereMujocoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kNumStates);

  for (size_t i = 0; i < kThrusterNames.size(); ++i) {
    interfaces.emplace_back(
      kThrusterNames[i], "effort", &hw_states_[kThrusterReadback + i]);
  }

  static constexpr std::array<const char *, 10> kImuNames = {
    "orientation.w", "orientation.x", "orientation.y", "orientation.z",
    "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
    "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};
  for (size_t i = 0; i < kImuNames.size(); ++i) {
    interfaces.emplace_back("imu_sensor", kImuNames[i], &hw_states_[kImu + i]);
  }

  static constexpr std::array<const char *, 10> kOdomNames = {
    "pos.x", "pos.y", "pos.z", "quat.w", "quat.x", "quat.y", "quat.z",
    "vel_body.x", "vel_body.y", "vel_body.z"};
  for (size_t i = 0; i < kOdomNames.size(); ++i) {
    interfaces.emplace_back("rov_odom", kOdomNames[i], &hw_states_[kOdom + i]);
  }

  static constexpr std::array<const char *, 8> kPadContactNames = {
    "pad_TL/force", "pad_TR/force", "pad_BL/force", "pad_BR/force",
    "pad_TL/v_sep", "pad_TR/v_sep", "pad_BL/v_sep", "pad_BR/v_sep"};
  for (size_t i = 0; i < kPadContactNames.size(); ++i) {
    interfaces.emplace_back("pad_contact", kPadContactNames[i], &hw_states_[kPadContact + i]);
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface> AdhereMujocoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kNumThrusters);
  for (size_t i = 0; i < kThrusterNames.size(); ++i) {
    interfaces.emplace_back(kThrusterNames[i], "effort", &hw_commands_[i]);
  }
  return interfaces;
}

hardware_interface::return_type AdhereMujocoHardware::start()
{
  if (!model_) {
    return hardware_interface::return_type::ERROR;
  }
  data_ = mj_makeData(model_);
  if (!data_) {
    return hardware_interface::return_type::ERROR;
  }

  mjcb_passive = [](const mjModel * m, mjData * d) {
    AdhereMujocoHardware * active = g_active_instance.load();
    if (active && active->data_ == d) {
      active->applyHydrodynamics(m, d);
    }
  };

  // Zero-delay two-stage MuJoCo step contract (CLAUDE.md Phase 2.6 Stage 3.1): mj_step1() here
  // seeds xipos/xquat/cvel/sensordata for the first read() call; write() then alternates
  // mj_step2() (finish current step) + mj_step1() (recompute kinematics/sensors for the new
  // state) so read() never observes 1-tick-stale derived quantities.
  mj_resetData(model_, data_);
  mj_step1(model_, data_);

  sim_time_ = 0.0;
  sim_time_atomic_.store(0.0);
  step_count_ = 0;
  have_prev_vel_ = false;
  last_accel_step_count_ = -1;
  wave_mode_.store(1);
  wave_trigger_time_.store(0.0);

  // Deferred from configure() (CLAUDE.md Phase 2.6 Stage 2.1): headless_ is only known now,
  // set by adhere_mujoco_node via setHeadless() between ResourceManager construction and
  // start_components().
  viewer_ = std::make_unique<MujocoViewer>(model_, vis_bridge_, headless_);
  viewer_->start();
  status_ = hardware_interface::status::STARTED;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AdhereMujocoHardware::stop()
{
  if (viewer_) {
    viewer_->stop();
    viewer_.reset();
  }
  if (g_active_instance.load() == this) {
    mjcb_passive = nullptr;
  }
  if (data_) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  status_ = hardware_interface::status::STOPPED;
  return hardware_interface::return_type::OK;
}

void AdhereMujocoHardware::setWaveMode(int mode)
{
  if (mode != 1 && mode != 2) {
    RCLCPP_WARN(logger(), "Ignoring invalid wave mode %d (expected 1=calm or 2=gust)", mode);
    return;
  }
  // Hydrodynamic-accuracy audit (thread-safety finding 1.1): wave_trigger_time_ must be
  // latched BEFORE wave_mode_ becomes externally visible as 2, not after. applyHydrodynamics()
  // (500Hz physics thread) and this call (ROS executor thread or the GUI's write()-internal
  // path) race against each other; with the old mode-then-trigger ordering, a tick landing in
  // the gap between the two stores could observe the new mode (2) paired with the still-stale
  // trigger time, corrupting that one tick's tau = t_sim - t_trigger computation. Storing the
  // trigger first closes that window: by the time wave_mode_ flips to 2, wave_trigger_time_ is
  // already correct.
  if (mode == 2) {
    wave_trigger_time_.store(sim_time_atomic_.load());
  }
  const int previous = wave_mode_.exchange(mode);
  if (previous != mode) {
    RCLCPP_INFO(
      logger(), "Wave mode switched: %s -> %s", previous == 2 ? "gust" : "calm",
      mode == 2 ? "gust" : "calm");
  }
}

AdhereMujocoHardware * AdhereMujocoHardware::activeInstance() { return g_active_instance.load(); }

void AdhereMujocoHardware::applyHydrodynamics(const mjModel * m, mjData * d)
{
  RigidBodyState state;
  state.com_pos_world = Eigen::Vector3d(
    d->xipos[3 * body_id_ + 0], d->xipos[3 * body_id_ + 1], d->xipos[3 * body_id_ + 2]);
  state.orientation = Eigen::Quaterniond(
    d->xquat[4 * body_id_ + 0], d->xquat[4 * body_id_ + 1], d->xquat[4 * body_id_ + 2],
    d->xquat[4 * body_id_ + 3]);
  state.ang_vel_world = Eigen::Vector3d(
    d->cvel[6 * body_id_ + 0], d->cvel[6 * body_id_ + 1], d->cvel[6 * body_id_ + 2]);
  state.lin_vel_world = Eigen::Vector3d(
    d->cvel[6 * body_id_ + 3], d->cvel[6 * body_id_ + 4], d->cvel[6 * body_id_ + 5]);

  // Relative wave clock (CLAUDE.md Phase 2.5 Stage 2.3/3.2): mode 2 evaluates the gust
  // profile at tau = t_sim - t_trigger (>= 0) instead of raw sim time, so switching into the
  // gust scenario at an arbitrary sim time always replays the gusts from their own t=0.
  const bool gust_active = wave_mode_.load() == 2;
  const HydroParams & params =
    gust_active ? gust_plugin_->getHydroParams() : calm_plugin_->getHydroParams();
  const double t_eval = gust_active ? std::max(0.0, d->time - wave_trigger_time_.load()) : d->time;

  HydroWrenchBreakdown breakdown;
  // Finding A: m->opt.timestep is this 500Hz physics thread's own fixed tick period (2.0ms),
  // the correct dt to integrate radiation_memory_state_'s washout filter forward by exactly one
  // step per mjcb_passive invocation.
  const Wrench wrench = FossenHydrodynamics::compute(
    state, t_eval, params, hydro_mode_.load(), &hydro_geometry_model_, &breakdown,
    &radiation_memory_state_, m->opt.timestep);

  // Live Hydro Breakdown telemetry (Phase 2.13 Section 2): plain atomic stores, same
  // lock-free pattern as kern_val/m_kern in write() below -- safe from this 500Hz physics-
  // thread callback without taking vis_bridge_->mtx.
  vis_bridge_->hydro_f_drag_y.store(breakdown.f_drag_body.y());
  vis_bridge_->hydro_f_venturi_x.store(breakdown.f_venturi_body.x());
  vis_bridge_->hydro_f_squeeze_x.store(breakdown.f_squeeze_body.x());
  vis_bridge_->hydro_f_fk_am_y.store(breakdown.f_fk_am_body.y());

  // Interactive mouse perturbation (CLAUDE.md Phase 2.6 Stage 3.1): MujocoViewer writes the
  // Ctrl+drag wrench it computed on its own secondary d_vis_ into vis_bridge_->xfrc_perturb;
  // folded into the same mj_applyFT call as the hydrodynamic wrench so both land in
  // qfrc_passive at the same point in the step.
  const mjtNum force[3] = {
    wrench.force_world.x() + vis_bridge_->xfrc_perturb[0].load(),
    wrench.force_world.y() + vis_bridge_->xfrc_perturb[1].load(),
    wrench.force_world.z() + vis_bridge_->xfrc_perturb[2].load()};
  const mjtNum torque[3] = {
    wrench.torque_world.x() + vis_bridge_->xfrc_perturb[3].load(),
    wrench.torque_world.y() + vis_bridge_->xfrc_perturb[4].load(),
    wrench.torque_world.z() + vis_bridge_->xfrc_perturb[5].load()};
  const mjtNum point[3] = {
    state.com_pos_world.x(), state.com_pos_world.y(), state.com_pos_world.z()};
  mj_applyFT(m, d, force, torque, point, body_id_, d->qfrc_passive);
}

hardware_interface::return_type AdhereMujocoHardware::read()
{
  if (!data_) {
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < kThrusterNames.size(); ++i) {
    hw_states_[kThrusterReadback + i] = data_->ctrl[actuator_id_[i]];
  }

  const Eigen::Quaterniond orientation(
    data_->xquat[4 * body_id_ + 0], data_->xquat[4 * body_id_ + 1],
    data_->xquat[4 * body_id_ + 2], data_->xquat[4 * body_id_ + 3]);
  const Eigen::Matrix3d R = orientation.toRotationMatrix();

  const Eigen::Vector3d ang_vel_world(
    data_->cvel[6 * body_id_ + 0], data_->cvel[6 * body_id_ + 1], data_->cvel[6 * body_id_ + 2]);
  const Eigen::Vector3d lin_vel_world(
    data_->cvel[6 * body_id_ + 3], data_->cvel[6 * body_id_ + 4], data_->cvel[6 * body_id_ + 5]);
  const Eigen::Vector3d ang_vel_body = R.transpose() * ang_vel_world;
  const Eigen::Vector3d lin_vel_body = R.transpose() * lin_vel_world;

  // IMU: orientation + body-frame gyro + finite-difference body-frame linear acceleration
  // (rov_wall.xml has no dedicated accelerometer sensor; this is a documented approximation).
  hw_states_[kImu + 0] = orientation.w();
  hw_states_[kImu + 1] = orientation.x();
  hw_states_[kImu + 2] = orientation.y();
  hw_states_[kImu + 3] = orientation.z();
  hw_states_[kImu + 4] = ang_vel_body.x();
  hw_states_[kImu + 5] = ang_vel_body.y();
  hw_states_[kImu + 6] = ang_vel_body.z();

  // Only recompute/advance the finite-difference accel estimate when a real physics step has
  // actually elapsed since the last read() (Phase 2.11 Section 3): read() runs every 500Hz tick
  // regardless of pause state, but step_count_ only advances when mj_step actually ran, so while
  // paused this latches the last valid accel_body_ instead of dividing a zero velocity delta by
  // dt and wiping the IMU reading to zero.
  if (step_count_ != last_accel_step_count_) {
    const double dt = model_->opt.timestep;
    Eigen::Vector3d accel_body = Eigen::Vector3d::Zero();
    if (have_prev_vel_ && dt > 0.0) {
      accel_body = R.transpose() * ((lin_vel_world - prev_lin_vel_world_) / dt);
    }
    last_accel_body_ = accel_body;
    prev_lin_vel_world_ = lin_vel_world;
    have_prev_vel_ = true;
    last_accel_step_count_ = step_count_;
  }
  hw_states_[kImu + 7] = last_accel_body_.x();
  hw_states_[kImu + 8] = last_accel_body_.y();
  hw_states_[kImu + 9] = last_accel_body_.z();

  // rov_odom: world-frame CoM position + orientation (quat) + body-frame linear velocity.
  hw_states_[kOdom + 0] = data_->xipos[3 * body_id_ + 0];
  hw_states_[kOdom + 1] = data_->xipos[3 * body_id_ + 1];
  hw_states_[kOdom + 2] = data_->xipos[3 * body_id_ + 2];
  hw_states_[kOdom + 3] = orientation.w();
  hw_states_[kOdom + 4] = orientation.x();
  hw_states_[kOdom + 5] = orientation.y();
  hw_states_[kOdom + 6] = orientation.z();
  hw_states_[kOdom + 7] = lin_vel_body.x();
  hw_states_[kOdom + 8] = lin_vel_body.y();
  hw_states_[kOdom + 9] = lin_vel_body.z();

  // pad_contact: normal force (touch sensor) + separation velocity, CLAUDE.md Sec.4.2:
  // v_sep,i = v_x,b + omega_y * z_i - omega_z * y_i. (y_i, z_i) come from pad_yz_, resolved at
  // configure() time from model_->site_pos -- never hardcoded (CLAUDE.md Phase 2.6 bug #6).
  for (size_t i = 0; i < kPadNames.size(); ++i) {
    hw_states_[kPadContact + i] = data_->sensordata[pad_touch_sensor_adr_[i]];
    const double y_i = pad_yz_[i].first;
    const double z_i = pad_yz_[i].second;
    hw_states_[kPadContact + 4 + i] =
      lin_vel_body.x() + ang_vel_body.y() * z_i - ang_vel_body.z() * y_i;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AdhereMujocoHardware::write()
{
  if (!data_) {
    return hardware_interface::return_type::ERROR;
  }

  // Increments every write() call regardless of pause state (CLAUDE.md Phase 2.10 Bug 2
  // fix): unlike step_count_ (the real physics-step counter, frozen while paused), this
  // keeps the vis-bridge snapshot cadence below alive even while paused, so "Step Once" and
  // manual-slider drags while paused are never stuck waiting for step_count_ to advance.
  ++write_tick_count_;

  // Hydro model mode selector (CLAUDE.md Phase 2.10 Section B9): plain pass-through, no
  // edge-detection needed -- always take whatever value the GUI radio currently holds.
  hydro_mode_.store(static_cast<HydroModelMode>(vis_bridge_->hydro_mode.load()));

  // GUI reset request (Backspace in MujocoViewer) takes priority over everything else this
  // tick: re-seed the authoritative mjData and re-latch mj_step1 so read() stays zero-delay.
  const bool did_reset = vis_bridge_->reset_request.exchange(false);
  if (did_reset) {
    mj_resetData(model_, data_);
    mj_step1(model_, data_);
    sim_time_ = data_->time;
    sim_time_atomic_.store(sim_time_);
    step_count_ = 0;
    have_prev_vel_ = false;
    last_accel_step_count_ = -1;
    // Phase 2.12 Item 5B: clear the first-order ESC lag filter state too, otherwise stale
    // lagged thrust from before the reset leaks into the first post-reset step whenever
    // enable_thruster_lag is active.
    applied_thrust_.fill(0.0);
    // Finding A: clear the radiation-memory washout filter state for the same reason -- stale
    // lagged_vel_body from before the reset must not leak into the first post-reset step.
    radiation_memory_state_ = RadiationMemoryState{};
  }

  // GUI "Reset Stats" request (Phase 2.11 Section 2), independent of the full physics reset
  // above: clears only the cumulative liftoff/corner-trap counters so a benchmark run can be
  // re-zeroed without resetting the ROV's pose/velocity.
  const bool reset_stats = did_reset || vis_bridge_->reset_stats_request.exchange(false);
  if (reset_stats) {
    total_eval_steps_ = 0;
    liftoff_steps_ = 0;
    liftoff_events_ = 0;
    corner_trap_steps_ = 0;
    prev_is_liftoff_ = false;
    vis_bridge_->liftoff_ratio.store(0.0);
    vis_bridge_->corner_trap_ratio.store(0.0);
    vis_bridge_->liftoff_events.store(0);
    vis_bridge_->active_pads.store(static_cast<int>(kPadNames.size()));
  }

  // GUI wave-mode request (1/2 keys in MujocoViewer): reuses the same setWaveMode() logic the
  // /rov/wave_mode_cmd topic subscriber calls, so both entry points stay consistent.
  const int wave_cmd = vis_bridge_->wave_mode_cmd.exchange(0);
  if (wave_cmd != 0) {
    setWaveMode(wave_cmd);
  }

  // Run/Pause (Space in MujocoViewer): when paused, skip stepping entirely so the scene stays
  // frozen at the last mj_step1()-settled state. "Step Once" (debugging: advance exactly one
  // 2.0ms tick while paused) is the one exception -- it consumes its own request flag so it
  // only ever fires once per button press/key tap, not every tick it stays true. Computed
  // before the thruster-write loop below (Phase 2.11 Section 4) since the optional lag filter
  // must only integrate forward when a physics step is actually about to occur.
  const bool paused = vis_bridge_->paused.load();
  const bool single_step = paused && vis_bridge_->step_once_request.exchange(false);
  const bool step_occurs = !paused || single_step;

  // hw_commands_ stays all-zero with no controller spawned (the "u=0 passive drift"
  // verification condition); manual_ctrl is the GUI's per-thruster slider override, additive
  // so a controller and manual testing never silently fight over the same channel without the
  // user noticing. AdhereMujocoHardware reads whatever value currently sits in hw_commands_
  // regardless of which rate last wrote it (ESC zero-order hold emulation per CLAUDE.md Sec.3).
  // Phase 2.11 Section 4 / Phase 2.12 Section 3: clamp to this thruster's own rail limit
  // (resolved from model_->actuator_ctrlrange at configure() time, not a hardcoded +-40N
  // constant) before applying, optionally snap to zero inside the ESC deadzone (CLAUDE.md
  // Sec.3.1), and optionally run a first-order ESC lag filter (tau_T = 15ms) instead of writing
  // the target thrust directly.
  constexpr double kTauThruster = 0.015;
  // Representative ESC null-band magnitude (~5% of the +-40N rail): below this |u_target|, a
  // real ESC outputs exactly zero thrust rather than a small linear response. Placeholder value
  // pending calibration against a real thruster's PWM-to-thrust curve.
  constexpr double kThrusterDeadzoneN = 2.0;
  const double dt = model_->opt.timestep;
  const double alpha_t = 1.0 - std::exp(-dt / kTauThruster);
  const bool lag_enabled = vis_bridge_->enable_thruster_lag.load();
  const bool deadzone_enabled = vis_bridge_->enable_thruster_deadzone.load();
  for (size_t i = 0; i < kThrusterNames.size(); ++i) {
    double u_target = std::clamp(
      hw_commands_[i] + vis_bridge_->manual_ctrl[i].load(), thruster_ctrl_min_[i],
      thruster_ctrl_max_[i]);
    if (deadzone_enabled) {
      u_target = applyThrusterDeadzone(u_target, kThrusterDeadzoneN);
    }
    if (lag_enabled) {
      if (step_occurs) {
        applied_thrust_[i] = applyThrusterLagStep(applied_thrust_[i], u_target, alpha_t);
      }
      // else: paused with lag enabled -- hold the last applied_thrust_ (no time has elapsed).
    } else {
      applied_thrust_[i] = u_target;
    }
    data_->ctrl[actuator_id_[i]] = applied_thrust_[i];
  }

  if (step_occurs) {
    mj_step2(model_, data_);
    mj_step1(model_, data_);
    sim_time_ = data_->time;
    sim_time_atomic_.store(sim_time_);
    ++step_count_;

    // True 500Hz aliasing-free contact & Rhombic-Kern metrics (Phase 2.11 Section 2): evaluated
    // on every real physics step (not just the ~60Hz vis-bridge snapshot cadence below), so
    // transient <5ms contact events are never missed. classifyRhombicKern() (hydrodynamic-
    // accuracy audit) factors the CoP/K_val/liftoff/corner-trap math into a unit-tested pure
    // function (test/test_rhombic_kern_and_thruster_model.cpp), including the L_y/L_z
    // divide-by-zero floor (finding 5.3).
    std::array<double, 4> pad_force_now{};
    for (size_t i = 0; i < kPadNames.size(); ++i) {
      pad_force_now[i] = data_->sensordata[pad_touch_sensor_adr_[i]];
    }
    const RhombicKernResult kern = classifyRhombicKern(pad_force_now, pad_yz_);

    ++total_eval_steps_;
    if (kern.is_liftoff) {
      ++liftoff_steps_;
      if (!prev_is_liftoff_) {
        ++liftoff_events_;
      }
    }
    if (kern.is_corner_trap) {
      ++corner_trap_steps_;
    }
    prev_is_liftoff_ = kern.is_liftoff;

    vis_bridge_->cop_y.store(kern.cop_y);
    vis_bridge_->cop_z.store(kern.cop_z);
    vis_bridge_->kern_val.store(kern.kern_val);
    vis_bridge_->m_kern.store(kern.m_kern);
    vis_bridge_->liftoff_ratio.store(
      static_cast<double>(liftoff_steps_) / static_cast<double>(total_eval_steps_));
    vis_bridge_->corner_trap_ratio.store(
      static_cast<double>(corner_trap_steps_) / static_cast<double>(total_eval_steps_));
    vis_bridge_->liftoff_events.store(liftoff_events_);
    vis_bridge_->active_pads.store(
      static_cast<int>(kPadNames.size()) - kern.liftoff_pad_count);
  }
  vis_bridge_->sim_time.store(sim_time_);

  // Every 8 write() ticks (~60Hz at dt=2ms wall-clock, now gated on write_tick_count_ instead
  // of step_count_ -- CLAUDE.md Phase 2.10 Bug 2 fix) -- or immediately on a single_step,
  // reset, wave-mode change, or any tick while paused (so a paused slider drag is visible on
  // its very next tick instead of waiting up to 7 ticks) -- snapshot state into the vis bridge
  // via try_lock() so a slow/blocked renderer can never stall this 500Hz loop.
  if (write_tick_count_ % 8 == 0 || single_step || did_reset || wave_cmd != 0 || paused) {
    std::unique_lock<std::mutex> lock(vis_bridge_->mtx, std::try_to_lock);
    if (lock.owns_lock()) {
      std::memcpy(vis_bridge_->qpos.data(), data_->qpos, vis_bridge_->qpos.size() * sizeof(double));
      std::memcpy(vis_bridge_->qvel.data(), data_->qvel, vis_bridge_->qvel.size() * sizeof(double));
      for (size_t i = 0; i < kThrusterNames.size(); ++i) {
        vis_bridge_->ctrl[i] = data_->ctrl[actuator_id_[i]];
      }
      for (size_t i = 0; i < kPadNames.size(); ++i) {
        vis_bridge_->pad_force[i] = data_->sensordata[pad_touch_sensor_adr_[i]];
        vis_bridge_->pad_v_sep[i] = hw_states_[kPadContact + 4 + i];
      }
      const bool gust_active = wave_mode_.load() == 2;
      const double t_eval =
        gust_active ? std::max(0.0, sim_time_ - wave_trigger_time_.load()) : sim_time_;
      const FlowEnvironmentPlugin & active_plugin =
        gust_active ? *gust_plugin_ : *calm_plugin_;
      const Eigen::Vector3d flow = active_plugin.ambientVelocityWorld(t_eval);
      vis_bridge_->flow_vel_world = {flow.x(), flow.y(), flow.z()};
      const Eigen::Vector3d flow_acc = active_plugin.ambientAccelerationWorld(t_eval);
      vis_bridge_->flow_acc_world = {flow_acc.x(), flow_acc.y(), flow_acc.z()};
      vis_bridge_->valid = true;
    }
    vis_bridge_->wave_mode_active.store(wave_mode_.load());
    vis_bridge_->wave_tau.store(
      wave_mode_.load() == 2 ? std::max(0.0, sim_time_ - wave_trigger_time_.load()) : 0.0);
    vis_bridge_->hydro_patch_count.store(static_cast<int>(hydro_geometry_model_.patches.size()));
  }

  return hardware_interface::return_type::OK;
}

}  // namespace adhere_mujoco_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  adhere_mujoco_hardware::AdhereMujocoHardware, hardware_interface::SystemInterface)
