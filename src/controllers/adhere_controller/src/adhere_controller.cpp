#include "adhere_controller/adhere_controller.hpp"

#include <algorithm>
#include <array>
#include <string>
#include <vector>

#include "adhere_controller/common/mathTools.h"

namespace adhere_controller
{

namespace
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Matches adhere_mujoco_hardware::kThrusterNames / adhere_mujoco_hardware.xacro's joint order.
constexpr std::array<const char *, 8> kThrusterNames = {
  "thr_N1", "thr_N2", "thr_N3", "thr_N4", "thr_T5", "thr_T6", "thr_T7", "thr_T8"};
constexpr std::array<const char *, 4> kPadNames = {"pad_TL", "pad_TR", "pad_BL", "pad_BR"};

std::vector<std::string> thrusterInterfaceNames()
{
  std::vector<std::string> names;
  names.reserve(kThrusterNames.size());
  for (const auto * name : kThrusterNames) {
    names.emplace_back(std::string(name) + "/effort");
  }
  return names;
}

std::vector<std::string> stateInterfaceNames()
{
  std::vector<std::string> names = thrusterInterfaceNames();

  for (const char * suffix :
       {"orientation.w", "orientation.x", "orientation.y", "orientation.z",
        "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
        "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"}) {
    names.emplace_back(std::string("imu_sensor/") + suffix);
  }

  for (const char * suffix :
       {"pos.x", "pos.y", "pos.z", "quat.w", "quat.x", "quat.y", "quat.z", "vel_body.x",
        "vel_body.y", "vel_body.z"}) {
    names.emplace_back(std::string("rov_odom/") + suffix);
  }

  for (const auto * pad : kPadNames) {
    names.emplace_back(std::string("pad_contact/") + pad + "/force");
  }
  for (const auto * pad : kPadNames) {
    names.emplace_back(std::string("pad_contact/") + pad + "/v_sep");
  }

  // Phase 3.5 realistic noisy multi-rate sensors (CLAUDE.md Stage 2.1/2.2): a second,
  // independent sensing path, bound to PRIVATE AdhereController members (never
  // CtrlInterfaces), consumed exclusively by RovStateEstimator.
  for (const char * suffix :
       {"orientation.w", "orientation.x", "orientation.y", "orientation.z",
        "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
        "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"}) {
    names.emplace_back(std::string("imu_sensor_noisy/") + suffix);
  }
  names.emplace_back("depth_sensor_noisy/z");
  for (const char * suffix : {"vel.x", "vel.y", "vel.z", "range"}) {
    names.emplace_back(std::string("dvl_sensor_noisy/") + suffix);
  }
  for (const auto * pad : kPadNames) {
    names.emplace_back(std::string("loadcell_noisy/") + pad + "/force");
  }

  return names;
}

// Finds the interface whose get_full_name() matches `full_name` in `interfaces` and appends a
// reference_wrapper to it onto `dest`. Interfaces are claimed via assign_interfaces() in
// whatever order controller_manager built them in, so this explicit full-name match (rather
// than assuming vector order mirrors the *_interface_configuration() request order) is what
// guarantees CtrlInterfaces' fixed slot layout regardless of that internal ordering.
template <typename InterfaceT, typename DestT>
bool findAndAppend(
  std::vector<InterfaceT> & interfaces, const std::string & full_name, DestT & dest)
{
  for (auto & iface : interfaces) {
    if (iface.get_full_name() == full_name) {
      dest.emplace_back(iface);
      return true;
    }
  }
  return false;
}

// Inverse of FSMState::checkFsmCommandChange()'s int -> FSMStateName mapping, used to populate
// FsmState.msg::current_state with the ACTIVE state (not whatever raw command value happens to
// be latched in ControlInputs, which may be stale or mid-transition).
int32_t fsmStateNameToInt(FSMStateName name)
{
  switch (name) {
    case FSMStateName::PASSIVE:
      return 1;
    case FSMStateName::DEPTH_HOLD:
      return 2;
    case FSMStateName::OPEN_LOOP:
      return 3;
    case FSMStateName::PID_HOLD:
      return 4;
    default:
      return 0;
  }
}

}  // namespace

controller_interface::InterfaceConfiguration
AdhereController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, thrusterInterfaceNames()};
}

controller_interface::InterfaceConfiguration
AdhereController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, stateInterfaceNames()};
}

controller_interface::return_type AdhereController::init(const std::string & controller_name)
{
  const auto base_result = controller_interface::ControllerInterface::init(controller_name);
  if (base_result != controller_interface::return_type::OK) {
    return base_result;
  }
  return controller_interface::return_type::OK;
}

CallbackReturn AdhereController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 4-topic decoupled contract (CLAUDE.md Phase 3.2 Stage 2.1): FsmCommand ONLY switches
  // states; SetpointCommand/TeleopCommand feed the live ControlInputs atomics consumed by the
  // active FSMState's run().
  fsm_command_subscription_ = get_node()->create_subscription<adhere_control_msgs::msg::FsmCommand>(
    "/rov/fsm_command", 10,
    [this](const adhere_control_msgs::msg::FsmCommand::SharedPtr msg) {
      ctrl_interfaces_.control_inputs_.command.store(msg->command);
    });

  setpoint_command_subscription_ =
    get_node()->create_subscription<adhere_control_msgs::msg::SetpointCommand>(
      "/rov/setpoint_cmd", 10,
      [this](const adhere_control_msgs::msg::SetpointCommand::SharedPtr msg) {
        auto & ci = ctrl_interfaces_.control_inputs_;
        ci.setpoint_command_type.store(msg->command_type);
        ci.setpoint_target_pos_x.store(msg->target_pose.position.x);
        ci.setpoint_target_pos_y.store(msg->target_pose.position.y);
        ci.setpoint_target_pos_z.store(msg->target_pose.position.z);
        ci.setpoint_target_quat_w.store(msg->target_pose.orientation.w);
        ci.setpoint_target_quat_x.store(msg->target_pose.orientation.x);
        ci.setpoint_target_quat_y.store(msg->target_pose.orientation.y);
        ci.setpoint_target_quat_z.store(msg->target_pose.orientation.z);
        // Bumped last (CLAUDE.md Phase 3.2 Stage 2.1's edge-detection contract): any FSMState
        // polling setpoint_seq only ever observes it change once every field above is already
        // up to date, never a half-written setpoint.
        ci.setpoint_seq.fetch_add(1);
      });

  teleop_command_subscription_ =
    get_node()->create_subscription<adhere_control_msgs::msg::TeleopCommand>(
      "/rov/teleop_cmd", 10,
      [this](const adhere_control_msgs::msg::TeleopCommand::SharedPtr msg) {
        auto & ci = ctrl_interfaces_.control_inputs_;
        ci.teleop_surge_x.store(msg->linear.x);
        ci.teleop_sway_y.store(msg->linear.y);
        ci.teleop_heave_z.store(msg->linear.z);
        ci.teleop_roll_rate.store(msg->angular.x);
        ci.teleop_pitch_rate.store(msg->angular.y);
        ci.teleop_yaw_rate.store(msg->angular.z);
        ci.teleop_active.store(msg->active);
        ci.teleop_last_rx_time_sec.store(get_node()->now().seconds());
      });

  controller_state_publisher_ = get_node()->create_publisher<adhere_control_msgs::msg::ControllerState>(
    "/rov/controller_state", 10);

  // Uniform integer-ratio multi-rate parameters (CLAUDE.md Phase 3.1.1 Stage 2.1), read here
  // (rather than on_activate()) so robot_control.yaml's adhere_controller.ros__parameters
  // block is consulted exactly once per configure, same lifecycle point as the subscription
  // above.
  cm_update_rate_ = auto_declare<int>("cm_update_rate", cm_update_rate_);
  inner_decimation_ = auto_declare<int>("inner_decimation", inner_decimation_);
  outer_decimation_ = auto_declare<int>("outer_decimation", outer_decimation_);

  // Model-decoupled parameters (CLAUDE.md Phase 3.3 Sec.3): loaded exactly once per configure,
  // same lifecycle point as cm_update_rate_/inner_decimation_/outer_decimation_ above.
  model_params_ = ModelParams::loadFromNode(get_node());

  decimator_ = std::make_shared<MultiRateDecimator>(
    cm_update_rate_, inner_decimation_, outer_decimation_);
  inner_period_ = rclcpp::Duration::from_seconds(decimator_->innerPeriodSeconds());

  return CallbackReturn::SUCCESS;
}

CallbackReturn AdhereController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  ctrl_interfaces_.clear();
  gt_imu_state_interface_.clear();
  gt_odom_state_interface_.clear();
  gt_pad_contact_state_interface_.clear();
  noisy_imu_state_interface_.clear();
  noisy_depth_state_interface_.clear();
  noisy_dvl_state_interface_.clear();
  noisy_loadcell_state_interface_.clear();
  estimator_initialized_ = false;

  for (const auto * name : kThrusterNames) {
    findAndAppend(
      command_interfaces_, std::string(name) + "/effort", ctrl_interfaces_.thruster_command_interface_);
  }
  for (const auto * name : kThrusterNames) {
    findAndAppend(
      state_interfaces_, std::string(name) + "/effort", ctrl_interfaces_.thruster_state_interface_);
  }
  for (const char * suffix :
       {"orientation.w", "orientation.x", "orientation.y", "orientation.z",
        "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
        "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"}) {
    findAndAppend(state_interfaces_, std::string("imu_sensor/") + suffix, gt_imu_state_interface_);
  }
  for (const char * suffix :
       {"pos.x", "pos.y", "pos.z", "quat.w", "quat.x", "quat.y", "quat.z", "vel_body.x",
        "vel_body.y", "vel_body.z"}) {
    findAndAppend(state_interfaces_, std::string("rov_odom/") + suffix, gt_odom_state_interface_);
  }
  for (const auto * pad : kPadNames) {
    findAndAppend(
      state_interfaces_, std::string("pad_contact/") + pad + "/force",
      gt_pad_contact_state_interface_);
  }
  for (const auto * pad : kPadNames) {
    findAndAppend(
      state_interfaces_, std::string("pad_contact/") + pad + "/v_sep",
      gt_pad_contact_state_interface_);
  }

  // Phase 3.5 realistic noisy multi-rate sensors: also private, never handed to CtrlInterfaces.
  for (const char * suffix :
       {"orientation.w", "orientation.x", "orientation.y", "orientation.z",
        "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
        "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"}) {
    findAndAppend(
      state_interfaces_, std::string("imu_sensor_noisy/") + suffix, noisy_imu_state_interface_);
  }
  findAndAppend(state_interfaces_, "depth_sensor_noisy/z", noisy_depth_state_interface_);
  for (const char * suffix : {"vel.x", "vel.y", "vel.z", "range"}) {
    findAndAppend(
      state_interfaces_, std::string("dvl_sensor_noisy/") + suffix, noisy_dvl_state_interface_);
  }
  for (const auto * pad : kPadNames) {
    findAndAppend(
      state_interfaces_, std::string("loadcell_noisy/") + pad + "/force",
      noisy_loadcell_state_interface_);
  }

  state_list_.passive = std::make_shared<StatePassive>(ctrl_interfaces_, model_params_);
  state_list_.depth_hold = std::make_shared<StateDepthHold>(ctrl_interfaces_, model_params_);
  state_list_.open_loop = std::make_shared<StateOpenLoop>(ctrl_interfaces_, model_params_);
  state_list_.pid_hold = std::make_shared<StatePidHold>(ctrl_interfaces_, model_params_);

  // Default-boots in PASSIVE (CLAUDE.md Phase 3.1 Stage 2.3): launching mujoco_sim.launch.py
  // immediately brings up the FSM in the safe zero-thrust state.
  current_state_ = state_list_.passive;
  current_state_->enter();
  next_state_ = current_state_;
  next_state_name_ = current_state_->state_name;
  mode_ = FSMMode::NORMAL;

  // Uniform integer-ratio multi-rate reset (CLAUDE.md Phase 3.1.1 Stage 2.1): tick 0 always
  // re-aligns inner_due and outer_due simultaneously, and CtrlInterfaces::frequency_ documents
  // the live inner-loop rate (cm_update_rate_/inner_decimation_) for any FSMState that wants to
  // read it instead of assuming a hardcoded constant.
  decimator_->reset();
  ctrl_interfaces_.frequency_ = cm_update_rate_ / inner_decimation_;

  return CallbackReturn::SUCCESS;
}

CallbackReturn AdhereController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  release_interfaces();
  ctrl_interfaces_.clear();
  gt_imu_state_interface_.clear();
  gt_odom_state_interface_.clear();
  gt_pad_contact_state_interface_.clear();
  noisy_imu_state_interface_.clear();
  noisy_depth_state_interface_.clear();
  noisy_dvl_state_interface_.clear();
  noisy_loadcell_state_interface_.clear();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AdhereController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn AdhereController::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn AdhereController::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type AdhereController::update()
{
  // ControllerManager calls update() on every 500Hz physics tick (CLAUDE.md Phase 3.1.1 Sec.3);
  // the decimator reports which of the uniform inner/outer schedules are due on this tick.
  const auto tick = decimator_->tick();

  if (!current_state_) {
    return controller_interface::return_type::OK;
  }

  const rclcpp::Time time = get_node()->now();

  // --- Phase 3.5/3.6: 500Hz dual state-acquisition, every physics tick, ahead of the inner-loop
  // gate (CLAUDE.md Stage 2.2/2.3). Ground truth is read here purely to seed/compare against
  // RovStateEstimator and to populate "ground_truth" mode's active_state copy (Phase 3.6: no
  // longer published by this controller at all -- /rov/ground_truth_odom is adhere_mujoco_node's
  // job) -- it is NEVER itself assigned into ctrl_interfaces_.active_state while in
  // "sensor_fusion" mode, so no FSMState ever observes it.
  const Vec3 gt_pos(
    gt_odom_state_interface_[0].get().get_value(), gt_odom_state_interface_[1].get().get_value(),
    gt_odom_state_interface_[2].get().get_value());
  const Quat gt_quat(
    gt_odom_state_interface_[3].get().get_value(), gt_odom_state_interface_[4].get().get_value(),
    gt_odom_state_interface_[5].get().get_value(), gt_odom_state_interface_[6].get().get_value());
  const Vec3 gt_vel_body(
    gt_odom_state_interface_[7].get().get_value(), gt_odom_state_interface_[8].get().get_value(),
    gt_odom_state_interface_[9].get().get_value());
  const Vec3 gt_omega_body(
    gt_imu_state_interface_[4].get().get_value(), gt_imu_state_interface_[5].get().get_value(),
    gt_imu_state_interface_[6].get().get_value());

  if (model_params_.estimation_mode == "sensor_fusion") {
    if (!estimator_initialized_) {
      // Latch the estimator to ground truth once on the first sensor_fusion tick (e.g. right
      // after on_activate()) purely so it does not have to converge from zero -- this is the
      // ONLY place ground truth touches the estimator's internal state, not an ongoing feed.
      estimator_.reset(gt_pos, gt_vel_body, gt_quat);
      estimator_initialized_ = true;
    }

    const double dt_phys = 1.0 / static_cast<double>(cm_update_rate_);
    const Quat noisy_quat(
      noisy_imu_state_interface_[0].get().get_value(), noisy_imu_state_interface_[1].get().get_value(),
      noisy_imu_state_interface_[2].get().get_value(), noisy_imu_state_interface_[3].get().get_value());
    const Vec3 gyro_meas(
      noisy_imu_state_interface_[4].get().get_value(), noisy_imu_state_interface_[5].get().get_value(),
      noisy_imu_state_interface_[6].get().get_value());
    const Vec3 accel_meas(
      noisy_imu_state_interface_[7].get().get_value(), noisy_imu_state_interface_[8].get().get_value(),
      noisy_imu_state_interface_[9].get().get_value());

    estimator_.predict(gyro_meas, accel_meas, dt_phys);
    estimator_.correctAttitude(noisy_quat);
    estimator_.correctDepth(
      noisy_depth_state_interface_[0].get().get_value(), model_params_.sensor_mount.r_depth);
    const Vec3 dvl_vel(
      noisy_dvl_state_interface_[0].get().get_value(), noisy_dvl_state_interface_[1].get().get_value(),
      noisy_dvl_state_interface_[2].get().get_value());
    estimator_.correctDvl(
      dvl_vel, noisy_dvl_state_interface_[3].get().get_value(), model_params_.sensor_mount.r_dvl);

    const Vec3 est_pos = estimator_.positionWorld();
    const Vec3 est_vel = estimator_.velocityBody();
    const Vec3 est_omega = estimator_.angularVelocityBody();
    const Quat est_quat = estimator_.orientation();
    const Vec3 est_rpy = estimator_.rpy();
    for (int i = 0; i < 3; ++i) {
      ctrl_interfaces_.active_state.pos_w[i] = est_pos(i);
      ctrl_interfaces_.active_state.rpy[i] = est_rpy(i);
      ctrl_interfaces_.active_state.vel_body[i] = est_vel(i);
      ctrl_interfaces_.active_state.omega_body[i] = est_omega(i);
    }
    for (int i = 0; i < 4; ++i) {
      ctrl_interfaces_.active_state.quat_w[i] = est_quat(i);
    }
    for (size_t i = 0; i < kPadNames.size(); ++i) {
      ctrl_interfaces_.active_state.pad_force[i] = noisy_loadcell_state_interface_[i].get().get_value();
      ctrl_interfaces_.active_state.v_sep[i] =
        est_vel.x() + est_omega.y() * model_params_.pad_pos[i].z() -
        est_omega.z() * model_params_.pad_pos[i].y();
    }
  } else {
    estimator_initialized_ = false;
    const Vec3 gt_rpy = rotMatToRPY(quatToRotMat(gt_quat));
    for (int i = 0; i < 3; ++i) {
      ctrl_interfaces_.active_state.pos_w[i] = gt_pos(i);
      ctrl_interfaces_.active_state.rpy[i] = gt_rpy(i);
      ctrl_interfaces_.active_state.vel_body[i] = gt_vel_body(i);
      ctrl_interfaces_.active_state.omega_body[i] = gt_omega_body(i);
    }
    for (int i = 0; i < 4; ++i) {
      ctrl_interfaces_.active_state.quat_w[i] = gt_quat(i);
    }
    for (size_t i = 0; i < kPadNames.size(); ++i) {
      ctrl_interfaces_.active_state.pad_force[i] = gt_pad_contact_state_interface_[i].get().get_value();
      ctrl_interfaces_.active_state.v_sep[i] = gt_pad_contact_state_interface_[4 + i].get().get_value();
    }
  }

  if (tick.inner_due) {
    // 0.25s /rov/teleop_cmd staleness watchdog (CLAUDE.md Phase 3.2 Stage 2.3): forces teleop
    // inputs to a safe all-zero/inactive state centrally, so no FSMState implementation needs
    // its own disconnect-handling logic.
    auto & ci = ctrl_interfaces_.control_inputs_;
    if (time.seconds() - ci.teleop_last_rx_time_sec.load() > kTeleopWatchdogSeconds) {
      ci.teleop_active.store(false);
      ci.teleop_surge_x.store(0.0);
      ci.teleop_sway_y.store(0.0);
      ci.teleop_heave_z.store(0.0);
      ci.teleop_yaw_rate.store(0.0);
      ci.teleop_pitch_rate.store(0.0);
      ci.teleop_roll_rate.store(0.0);
    }

    if (mode_ == FSMMode::NORMAL) {
      current_state_->run(time, inner_period_);
      next_state_name_ = current_state_->checkChange();
      if (next_state_name_ != current_state_->state_name) {
        mode_ = FSMMode::CHANGE;
        next_state_ = getNextState(next_state_name_);
      }
    } else {
      current_state_->exit();
      current_state_ = next_state_;
      current_state_->enter();
      mode_ = FSMMode::NORMAL;
    }
  }

  if (tick.outer_due) {
    // Outer-loop hook (default 25Hz, CLAUDE.md Phase 3.1.1 Stage 2.1): the async CUDA MPPI
    // rollout thread hand-off (qpos, qvel, F_x_extra_preload) <-> w_cmd via
    // realtime_tools::RealtimeBuffer is a Phase 4/5 milestone -- intentionally a no-op placeholder
    // here so the uniform timing contract itself is already wired and verifiable (see
    // adhere_controller's test_multirate_decimator).

    // Live terminal feedback (CLAUDE.md Phase 3.6's ControllerState.msg, 100% Sim-to-Real pure
    // -- no ground truth here at all): published here (not gated on tick.inner_due) so
    // fsm_console_node/state_cmd_console_node see live estimated_pose/target_pose at the
    // outer-loop rate regardless of which inner-loop state is active.
    adhere_control_msgs::msg::ControllerState controller_state_msg;
    controller_state_msg.header.stamp = time;
    controller_state_msg.current_state = fsmStateNameToInt(current_state_->state_name);
    controller_state_msg.state_name = current_state_->state_name_string;
    controller_state_msg.estimation_mode = model_params_.estimation_mode;

    const auto target = current_state_->targetPose();
    controller_state_msg.target_pose.position.x = target.pos.x();
    controller_state_msg.target_pose.position.y = target.pos.y();
    controller_state_msg.target_pose.position.z = target.pos.z();
    controller_state_msg.target_pose.orientation.w = target.quat(0);
    controller_state_msg.target_pose.orientation.x = target.quat(1);
    controller_state_msg.target_pose.orientation.y = target.quat(2);
    controller_state_msg.target_pose.orientation.z = target.quat(3);

    const auto & active = ctrl_interfaces_.active_state;
    controller_state_msg.estimated_pose.position.x = active.pos_w[0];
    controller_state_msg.estimated_pose.position.y = active.pos_w[1];
    controller_state_msg.estimated_pose.position.z = active.pos_w[2];
    controller_state_msg.estimated_pose.orientation.w = active.quat_w[0];
    controller_state_msg.estimated_pose.orientation.x = active.quat_w[1];
    controller_state_msg.estimated_pose.orientation.y = active.quat_w[2];
    controller_state_msg.estimated_pose.orientation.z = active.quat_w[3];
    controller_state_msg.estimated_twist_body.linear.x = active.vel_body[0];
    controller_state_msg.estimated_twist_body.linear.y = active.vel_body[1];
    controller_state_msg.estimated_twist_body.linear.z = active.vel_body[2];
    controller_state_msg.estimated_twist_body.angular.x = active.omega_body[0];
    controller_state_msg.estimated_twist_body.angular.y = active.omega_body[1];
    controller_state_msg.estimated_twist_body.angular.z = active.omega_body[2];
    for (int i = 0; i < 4; ++i) {
      controller_state_msg.contact_forces[i] = active.pad_force[i];
    }
    for (int i = 0; i < 8; ++i) {
      controller_state_msg.thruster_cmds[i] =
        ctrl_interfaces_.thruster_state_interface_[i].get().get_value();
    }
    // kern_margin (Rhombic Kern safety margin, CLAUDE.md Sec.4.4) requires the A_contact/C_kern
    // precompute landing in Phase 3.6.3's ThrusterAllocator extension -- placeholder until then.
    controller_state_msg.kern_margin = 0.0;

    controller_state_publisher_->publish(controller_state_msg);
  }

  return controller_interface::return_type::OK;
}

std::shared_ptr<FSMState> AdhereController::getNextState(
  FSMStateName state_name) const
{
  switch (state_name) {
    case FSMStateName::PASSIVE:
      return state_list_.passive;
    case FSMStateName::DEPTH_HOLD:
      return state_list_.depth_hold;
    case FSMStateName::OPEN_LOOP:
      return state_list_.open_loop;
    case FSMStateName::PID_HOLD:
      return state_list_.pid_hold;
    default:
      return state_list_.passive;
  }
}

}  // namespace adhere_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  adhere_controller::AdhereController, controller_interface::ControllerInterface)
