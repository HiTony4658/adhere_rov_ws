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
  last_update_time_ = get_node()->now();

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
  if (!current_state_) {
    return controller_interface::return_type::OK;
  }

  const rclcpp::Time time = get_node()->now();
  // Foxy's ControllerInterface::update() takes no time/period arguments (CLAUDE.md Sec.2):
  // period is measured directly from consecutive update() calls' get_node()->now() instead of
  // assuming a constant 1/frequency_ step, so it stays correct even under scheduling jitter.
  const rclcpp::Duration period = time - last_update_time_;
  last_update_time_ = time;

  if (mode_ == adhere_controller_common::FSMMode::NORMAL) {
    current_state_->run(time, period);
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
