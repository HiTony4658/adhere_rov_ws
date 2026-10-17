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
        ci.setpoint_mode.store(msg->mode);
        ci.setpoint_lock_current_pose.store(msg->lock_current_pose);
        ci.setpoint_target_x.store(msg->target_x);
        ci.setpoint_target_y.store(msg->target_y);
        ci.setpoint_target_z.store(msg->target_z);
        ci.setpoint_target_yaw.store(msg->target_yaw);
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
        ci.teleop_surge_x.store(msg->surge_x);
        ci.teleop_sway_y.store(msg->sway_y);
        ci.teleop_heave_z.store(msg->heave_z);
        ci.teleop_yaw_rate.store(msg->yaw_rate);
        ci.teleop_active.store(msg->active);
        ci.teleop_last_rx_time_sec.store(get_node()->now().seconds());
      });

  fsm_state_publisher_ =
    get_node()->create_publisher<adhere_control_msgs::msg::FsmState>("/rov/fsm_state", 10);

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
    findAndAppend(
      state_interfaces_, std::string("imu_sensor/") + suffix, ctrl_interfaces_.imu_state_interface_);
  }
  for (const char * suffix :
       {"pos.x", "pos.y", "pos.z", "quat.w", "quat.x", "quat.y", "quat.z", "vel_body.x",
        "vel_body.y", "vel_body.z"}) {
    findAndAppend(
      state_interfaces_, std::string("rov_odom/") + suffix, ctrl_interfaces_.odom_state_interface_);
  }
  for (const auto * pad : kPadNames) {
    findAndAppend(
      state_interfaces_, std::string("pad_contact/") + pad + "/force",
      ctrl_interfaces_.pad_contact_state_interface_);
  }
  for (const auto * pad : kPadNames) {
    findAndAppend(
      state_interfaces_, std::string("pad_contact/") + pad + "/v_sep",
      ctrl_interfaces_.pad_contact_state_interface_);
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

    // Live terminal feedback (CLAUDE.md Phase 3.2 Stage 2.1's FsmState.msg): published here
    // (not gated on tick.inner_due) so fsm_console_node/state_cmd_console_node see live
    // actual_pose/target_pose at the outer-loop rate regardless of which inner-loop state is
    // active.
    adhere_control_msgs::msg::FsmState fsm_state_msg;
    fsm_state_msg.header.stamp = time;
    fsm_state_msg.current_state = fsmStateNameToInt(current_state_->state_name);
    fsm_state_msg.state_name = current_state_->state_name_string;

    const auto target_pose = current_state_->targetPose();
    std::copy(target_pose.begin(), target_pose.end(), fsm_state_msg.target_pose.begin());

    const double x = ctrl_interfaces_.odom_state_interface_[0].get().get_value();
    const double y = ctrl_interfaces_.odom_state_interface_[1].get().get_value();
    const double z = ctrl_interfaces_.odom_state_interface_[2].get().get_value();
    const Quat quat(
      ctrl_interfaces_.odom_state_interface_[3].get().get_value(),
      ctrl_interfaces_.odom_state_interface_[4].get().get_value(),
      ctrl_interfaces_.odom_state_interface_[5].get().get_value(),
      ctrl_interfaces_.odom_state_interface_[6].get().get_value());
    const double yaw =
      rotMatToRPY(quatToRotMat(quat))(2);
    fsm_state_msg.actual_pose = {x, y, z, yaw};

    fsm_state_publisher_->publish(fsm_state_msg);
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
