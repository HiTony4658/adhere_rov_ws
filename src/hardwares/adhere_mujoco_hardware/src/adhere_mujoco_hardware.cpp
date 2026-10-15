#include "adhere_mujoco_hardware/adhere_mujoco_hardware.hpp"

#include <algorithm>
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
AdhereMujocoHardware * g_active_instance = nullptr;

bool parseBoolParam(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  bool default_value)
{
  const auto it = params.find(key);
  if (it == params.end()) {
    return default_value;
  }
  return it->second == "true" || it->second == "1";
}
}  // namespace

AdhereMujocoHardware::~AdhereMujocoHardware()
{
  stop();
  if (g_active_instance == this) {
    g_active_instance = nullptr;
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

  const bool headless = parseBoolParam(info_.hardware_parameters, "headless", false);

  const std::string pkg_share =
    ament_index_cpp::get_package_share_directory("simplified_v1_description");
  const std::string mjcf_path = pkg_share + "/mjcf/rov_wall.xml";

  char error[1000] = "";
  model_ = mj_loadXML(mjcf_path.c_str(), nullptr, error, sizeof(error));
  if (!model_) {
    RCLCPP_ERROR(logger(), "Failed to load MJCF '%s': %s", mjcf_path.c_str(), error);
    return hardware_interface::return_type::ERROR;
  }

  body_id_ = mj_name2id(model_, mjOBJ_BODY, "rov_base");
  if (body_id_ < 0) {
    RCLCPP_ERROR(logger(), "Body 'rov_base' not found in MJCF");
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < kPadNames.size(); ++i) {
    const std::string sensor_name = std::string("touch_") + kPadNames[i];
    const int sensor_id = mj_name2id(model_, mjOBJ_SENSOR, sensor_name.c_str());
    if (sensor_id < 0) {
      RCLCPP_ERROR(logger(), "Sensor '%s' not found in MJCF", sensor_name.c_str());
      return hardware_interface::return_type::ERROR;
    }
    pad_touch_sensor_adr_[i] = model_->sensor_adr[sensor_id];
  }

  for (size_t i = 0; i < kThrusterNames.size(); ++i) {
    const int act_id = mj_name2id(model_, mjOBJ_ACTUATOR, kThrusterNames[i]);
    if (act_id < 0) {
      RCLCPP_ERROR(logger(), "Actuator '%s' not found in MJCF", kThrusterNames[i]);
      return hardware_interface::return_type::ERROR;
    }
    actuator_id_[i] = act_id;
  }

  // Always starts in Calm Water (CLAUDE.md Phase 2.5 Stage 2.3): both plugins are constructed
  // up front and selected at runtime via setWaveMode(), never by a static launch-time param.
  calm_plugin_ = std::make_unique<CalmWaterPlugin>();
  gust_plugin_ = std::make_unique<StationKeepingGustPlugin>();
  wave_mode_.store(1);
  wave_trigger_time_.store(0.0);
  RCLCPP_INFO(
    logger(), "AdhereMujocoHardware: starting in Calm Water mode, headless=%s",
    headless ? "true" : "false");

  vis_bridge_ = std::make_shared<MujocoVisBridge>(model_->nq, model_->nv);
  viewer_ = std::make_unique<MujocoViewer>(model_, vis_bridge_, headless);

  hw_states_.fill(0.0);
  hw_commands_.fill(0.0);

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
  mj_forward(model_, data_);

  g_active_instance = this;
  mjcb_passive = [](const mjModel * m, mjData * d) {
    if (g_active_instance && g_active_instance->data_ == d) {
      g_active_instance->applyHydrodynamics(m, d);
    }
  };

  sim_time_ = 0.0;
  sim_time_atomic_.store(0.0);
  step_count_ = 0;
  have_prev_vel_ = false;
  wave_mode_.store(1);
  wave_trigger_time_.store(0.0);

  viewer_->start();
  status_ = hardware_interface::status::STARTED;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AdhereMujocoHardware::stop()
{
  if (viewer_) {
    viewer_->stop();
  }
  if (g_active_instance == this) {
    mjcb_passive = nullptr;
    g_active_instance = nullptr;
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
  const int previous = wave_mode_.exchange(mode);
  if (mode == 2) {
    // Whenever mode 2 is (re-)selected, latch t_trigger to the current sim time so the gust
    // profile always replays from its own tau=0, no matter when the user pressed "2".
    wave_trigger_time_.store(sim_time_atomic_.load());
  }
  if (previous != mode) {
    RCLCPP_INFO(
      logger(), "Wave mode switched: %s -> %s", previous == 2 ? "gust" : "calm",
      mode == 2 ? "gust" : "calm");
  }
}

AdhereMujocoHardware * AdhereMujocoHardware::activeInstance() { return g_active_instance; }

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

  const Wrench wrench = FossenHydrodynamics::compute(state, t_eval, params);

  const mjtNum force[3] = {wrench.force_world.x(), wrench.force_world.y(), wrench.force_world.z()};
  const mjtNum torque[3] = {
    wrench.torque_world.x(), wrench.torque_world.y(), wrench.torque_world.z()};
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

  const double dt = model_->opt.timestep;
  Eigen::Vector3d accel_body = Eigen::Vector3d::Zero();
  if (have_prev_vel_ && dt > 0.0) {
    accel_body = R.transpose() * ((lin_vel_world - prev_lin_vel_world_) / dt);
  }
  hw_states_[kImu + 7] = accel_body.x();
  hw_states_[kImu + 8] = accel_body.y();
  hw_states_[kImu + 9] = accel_body.z();
  prev_lin_vel_world_ = lin_vel_world;
  have_prev_vel_ = true;

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
  // v_sep,i = v_x,b + omega_y * z_i - omega_z * y_i.
  static constexpr std::array<std::pair<double, double>, 4> kPadYZ = {
    std::make_pair(0.25, 0.15), std::make_pair(-0.25, 0.15), std::make_pair(0.25, -0.15),
    std::make_pair(-0.25, -0.15)};
  for (size_t i = 0; i < kPadNames.size(); ++i) {
    hw_states_[kPadContact + i] = data_->sensordata[pad_touch_sensor_adr_[i]];
    const double y_i = kPadYZ[i].first;
    const double z_i = kPadYZ[i].second;
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

  // hw_commands_ stays all-zero with no controller spawned -- this is exactly the "u=0
  // passive drift" condition this phase verifies. AdhereMujocoHardware reads whatever value
  // currently sits in hw_commands_ regardless of which rate last wrote it (ESC zero-order
  // hold emulation per CLAUDE.md Sec.3).
  for (size_t i = 0; i < kThrusterNames.size(); ++i) {
    data_->ctrl[actuator_id_[i]] = hw_commands_[i];
  }

  mj_step(model_, data_);
  sim_time_ = data_->time;
  sim_time_atomic_.store(sim_time_);
  ++step_count_;

  // Every 8 steps (~60Hz at dt=2ms), snapshot state into the vis bridge via try_lock() so a
  // slow/blocked renderer can never stall this 500Hz loop.
  if (step_count_ % 8 == 0) {
    std::unique_lock<std::mutex> lock(vis_bridge_->mtx, std::try_to_lock);
    if (lock.owns_lock()) {
      std::memcpy(vis_bridge_->qpos.data(), data_->qpos, vis_bridge_->qpos.size() * sizeof(double));
      std::memcpy(vis_bridge_->qvel.data(), data_->qvel, vis_bridge_->qvel.size() * sizeof(double));
      for (size_t i = 0; i < kThrusterNames.size(); ++i) {
        vis_bridge_->ctrl[i] = data_->ctrl[actuator_id_[i]];
      }
      for (size_t i = 0; i < kPadNames.size(); ++i) {
        vis_bridge_->pad_force[i] = data_->sensordata[pad_touch_sensor_adr_[i]];
      }
      const bool gust_active = wave_mode_.load() == 2;
      const double t_eval =
        gust_active ? std::max(0.0, sim_time_ - wave_trigger_time_.load()) : sim_time_;
      const FlowEnvironmentPlugin & active_plugin =
        gust_active ? *gust_plugin_ : *calm_plugin_;
      const Eigen::Vector3d flow = active_plugin.ambientVelocityWorld(t_eval);
      vis_bridge_->flow_vel_world = {flow.x(), flow.y(), flow.z()};
      vis_bridge_->valid = true;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace adhere_mujoco_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  adhere_mujoco_hardware::AdhereMujocoHardware, hardware_interface::SystemInterface)
