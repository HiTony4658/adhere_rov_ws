#ifndef ADHERE_KAMR_CONTROLLER__ADHERE_KAMR_CONTROLLER_HPP_
#define ADHERE_KAMR_CONTROLLER__ADHERE_KAMR_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "controller_interface/controller_interface.hpp"

#include "adhere_control_msgs/msg/fsm_command.hpp"
#include "adhere_controller_common/common/enumClass.h"
#include "adhere_controller_common/CtrlInterfaces.h"
#include "adhere_controller_common/FSM/FSMState.h"
#include "adhere_controller_common/FSM/StateDepthHold.h"
#include "adhere_controller_common/FSM/StatePassive.h"

namespace adhere_kamr_controller
{

// AdhereKamrController: the ROS 2 Foxy controller_interface::ControllerInterface plugin that
// owns the FSM (CLAUDE.md Phase 3.1 Stage 2.3). Foxy has no on_init()/update(time, period)
// overrides -- init(const std::string&) and update() (no args) are the real virtuals, so time
// and period are computed here from get_node()->now() before being forwarded to
// FSMState::run(time, period), exactly as CLAUDE.md Sec.2 specifies.
//
// Future KAMR-MPPI milestones (200Hz inner-loop Impedance PD + Pad-Force DOB + 12D OSQP, 20Hz
// async CUDA outer loop) add further FSMState subclasses to this same state machine; Phase 3.1
// only wires PASSIVE and DEPTH_HOLD.
class AdhereKamrController : public controller_interface::ControllerInterface
{
public:
  AdhereKamrController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type init(const std::string & controller_name) override;

  controller_interface::return_type update() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  struct FSMStateList
  {
    std::shared_ptr<adhere_controller_common::StatePassive> passive;
    std::shared_ptr<adhere_controller_common::StateDepthHold> depth_hold;
  };

  std::shared_ptr<adhere_controller_common::FSMState> getNextState(
    adhere_controller_common::FSMStateName state_name) const;

  adhere_controller_common::CtrlInterfaces ctrl_interfaces_;

  adhere_controller_common::FSMMode mode_ = adhere_controller_common::FSMMode::NORMAL;
  FSMStateList state_list_;
  std::shared_ptr<adhere_controller_common::FSMState> current_state_;
  std::shared_ptr<adhere_controller_common::FSMState> next_state_;
  adhere_controller_common::FSMStateName next_state_name_ =
    adhere_controller_common::FSMStateName::INVALID;

  rclcpp::Subscription<adhere_control_msgs::msg::FsmCommand>::SharedPtr fsm_command_subscription_;

  // Measured directly from consecutive update() calls (Foxy's ControllerInterface::update()
  // takes no time/period arguments, CLAUDE.md Sec.2), reset in on_activate() so a
  // deactivate/reactivate cycle never produces one huge stale-period tick.
  rclcpp::Time last_update_time_;
};

}  // namespace adhere_kamr_controller

#endif  // ADHERE_KAMR_CONTROLLER__ADHERE_KAMR_CONTROLLER_HPP_
