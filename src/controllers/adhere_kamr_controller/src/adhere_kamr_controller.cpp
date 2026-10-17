#include "adhere_kamr_controller/adhere_kamr_controller.hpp"

#include <array>
#include <string>
#include <vector>

namespace adhere_kamr_controller
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

}  // namespace

controller_interface::InterfaceConfiguration
AdhereKamrController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, thrusterInterfaceNames()};
}

controller_interface::InterfaceConfiguration
AdhereKamrController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, stateInterfaceNames()};
}

controller_interface::return_type AdhereKamrController::init(const std::string & controller_name)
{
  const auto base_result = controller_interface::ControllerInterface::init(controller_name);
  if (base_result != controller_interface::return_type::OK) {
    return base_result;
  }
  return controller_interface::return_type::OK;
}

CallbackReturn AdhereKamrController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  fsm_command_subscription_ = get_node()->create_subscription<adhere_control_msgs::msg::FsmCommand>(
    "/rov/fsm_command", 10,
    [this](const adhere_control_msgs::msg::FsmCommand::SharedPtr msg) {
      ctrl_interfaces_.control_inputs_.command.store(msg->command);
      ctrl_interfaces_.control_inputs_.target_z.store(msg->target_z);
      ctrl_interfaces_.control_inputs_.target_y.store(msg->target_y);
      ctrl_interfaces_.control_inputs_.target_yaw.store(msg->target_yaw);
    });

  // Uniform integer-ratio multi-rate parameters (CLAUDE.md Phase 3.1.1 Stage 2.1), read here
  // (rather than on_activate()) so robot_control.yaml's adhere_kamr_controller.ros__parameters
  // block is consulted exactly once per configure, same lifecycle point as the subscription
  // above.
  cm_update_rate_ = auto_declare<int>("cm_update_rate", cm_update_rate_);
  inner_decimation_ = auto_declare<int>("inner_decimation", inner_decimation_);
  outer_decimation_ = auto_declare<int>("outer_decimation", outer_decimation_);

  decimator_ = std::make_shared<adhere_controller_common::MultiRateDecimator>(
    cm_update_rate_, inner_decimation_, outer_decimation_);
  inner_period_ = rclcpp::Duration::from_seconds(decimator_->innerPeriodSeconds());

  return CallbackReturn::SUCCESS;
}

CallbackReturn AdhereKamrController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
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

  state_list_.passive = std::make_shared<adhere_controller_common::StatePassive>(ctrl_interfaces_);
  state_list_.depth_hold =
    std::make_shared<adhere_controller_common::StateDepthHold>(ctrl_interfaces_);

  // Default-boots in PASSIVE (CLAUDE.md Phase 3.1 Stage 2.3): launching mujoco_sim.launch.py
  // immediately brings up the FSM in the safe zero-thrust state.
  current_state_ = state_list_.passive;
  current_state_->enter();
  next_state_ = current_state_;
  next_state_name_ = current_state_->state_name;
  mode_ = adhere_controller_common::FSMMode::NORMAL;

  // Uniform integer-ratio multi-rate reset (CLAUDE.md Phase 3.1.1 Stage 2.1): tick 0 always
  // re-aligns inner_due and outer_due simultaneously, and CtrlInterfaces::frequency_ documents
  // the live inner-loop rate (cm_update_rate_/inner_decimation_) for any FSMState that wants to
  // read it instead of assuming a hardcoded constant.
  decimator_->reset();
  ctrl_interfaces_.frequency_ = cm_update_rate_ / inner_decimation_;

  return CallbackReturn::SUCCESS;
}

CallbackReturn AdhereKamrController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  release_interfaces();
  ctrl_interfaces_.clear();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AdhereKamrController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn AdhereKamrController::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn AdhereKamrController::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type AdhereKamrController::update()
{
  // ControllerManager calls update() on every 500Hz physics tick (CLAUDE.md Phase 3.1.1 Sec.3);
  // the decimator reports which of the uniform inner/outer schedules are due on this tick.
  const auto tick = decimator_->tick();

  if (tick.outer_due) {
    // Outer-loop hook (default 25Hz, CLAUDE.md Phase 3.1.1 Stage 2.1): the async CUDA MPPI
    // rollout thread hand-off (qpos, qvel, F_x_extra_preload) <-> w_cmd via
    // realtime_tools::RealtimeBuffer is a Phase 4/5 milestone -- intentionally a no-op placeholder
    // here so the uniform timing contract itself is already wired and verifiable (see
    // adhere_controller_common's test_multirate_decimator).
  }

  if (!tick.inner_due) {
    // Intermediate 500Hz tick (CLAUDE.md Phase 3.1.1 Stage 2.1): leave
    // thruster_command_interface_ untouched so AdhereMujocoHardware::write() holds the previous
    // inner-loop command (exact ESC Zero-Order Hold).
    return controller_interface::return_type::OK;
  }

  if (!current_state_) {
    return controller_interface::return_type::OK;
  }

  const rclcpp::Time time = get_node()->now();
  if (mode_ == adhere_controller_common::FSMMode::NORMAL) {
    current_state_->run(time, inner_period_);
    next_state_name_ = current_state_->checkChange();
    if (next_state_name_ != current_state_->state_name) {
      mode_ = adhere_controller_common::FSMMode::CHANGE;
      next_state_ = getNextState(next_state_name_);
    }
  } else {
    current_state_->exit();
    current_state_ = next_state_;
    current_state_->enter();
    mode_ = adhere_controller_common::FSMMode::NORMAL;
  }

  return controller_interface::return_type::OK;
}

std::shared_ptr<adhere_controller_common::FSMState> AdhereKamrController::getNextState(
  adhere_controller_common::FSMStateName state_name) const
{
  switch (state_name) {
    case adhere_controller_common::FSMStateName::PASSIVE:
      return state_list_.passive;
    case adhere_controller_common::FSMStateName::DEPTH_HOLD:
      return state_list_.depth_hold;
    default:
      return state_list_.passive;
  }
}

}  // namespace adhere_kamr_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  adhere_kamr_controller::AdhereKamrController, controller_interface::ControllerInterface)
