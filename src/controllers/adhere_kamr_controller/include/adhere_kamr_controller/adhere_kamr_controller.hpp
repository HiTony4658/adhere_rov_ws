#ifndef ADHERE_KAMR_CONTROLLER__ADHERE_KAMR_CONTROLLER_HPP_
#define ADHERE_KAMR_CONTROLLER__ADHERE_KAMR_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "controller_interface/controller_interface.hpp"

#include "adhere_control_msgs/msg/fsm_command.hpp"
#include "adhere_control_msgs/msg/fsm_state.hpp"
#include "adhere_control_msgs/msg/setpoint_command.hpp"
#include "adhere_control_msgs/msg/teleop_command.hpp"
#include "adhere_controller_common/common/enumClass.h"
#include "adhere_controller_common/common/MultiRateDecimator.h"
#include "adhere_controller_common/CtrlInterfaces.h"
#include "adhere_controller_common/FSM/FSMState.h"
#include "adhere_controller_common/FSM/StateDepthHold.h"
#include "adhere_controller_common/FSM/StateOpenLoop.h"
#include "adhere_controller_common/FSM/StatePassive.h"
#include "adhere_controller_common/FSM/StatePidHold.h"

namespace adhere_kamr_controller
{

// AdhereKamrController: the ROS 2 Foxy controller_interface::ControllerInterface plugin that
// owns the FSM (CLAUDE.md Phase 3.1 Stage 2.3). Foxy has no on_init()/update(time, period)
// overrides -- init(const std::string&) and update() (no args) are the real virtuals, called by
// ControllerManager on every 500Hz physics tick (CLAUDE.md Phase 3.1.1 Sec.3). A
// MultiRateDecimator divides that master clock into a uniform inner_decimation_-ratio inner loop
// (default 2 -> 250Hz, FSMState::run(time, period) with a *constant* period, zero
// 4ms/6ms-style alternation) and an outer_decimation_-ratio outer-loop hook (default 20 -> 25Hz),
// replacing the former non-uniform [3,3,3,3,3,2,2,2,2,2] substep schedule.
//
// Future KAMR-MPPI milestones (Impedance PD + Pad-Force DOB + 12D OSQP on the inner loop, async
// CUDA MPPI rollout on the outer-loop hook) add further FSMState subclasses to this same state
// machine; Phase 3.1 only wires PASSIVE and DEPTH_HOLD.
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
    std::shared_ptr<adhere_controller_common::StateOpenLoop> open_loop;
    std::shared_ptr<adhere_controller_common::StatePidHold> pid_hold;
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
  rclcpp::Subscription<adhere_control_msgs::msg::SetpointCommand>::SharedPtr
    setpoint_command_subscription_;
  rclcpp::Subscription<adhere_control_msgs::msg::TeleopCommand>::SharedPtr
    teleop_command_subscription_;
  rclcpp::Publisher<adhere_control_msgs::msg::FsmState>::SharedPtr fsm_state_publisher_;

  // 0.25s /rov/teleop_cmd staleness watchdog (CLAUDE.md Phase 3.2 Stage 2.3): compared against
  // ctrl_interfaces_.control_inputs_.teleop_last_rx_time_sec on every inner-due tick, in
  // get_node()->now().seconds() units (consistent with how the subscription callback stamps
  // it) so no clock-type mismatch is possible.
  static constexpr double kTeleopWatchdogSeconds = 0.25;

  // Uniform integer-ratio multi-rate decimation (CLAUDE.md Phase 3.1.1 Stage 2.1): cm_update_rate_
  // is the master controller_manager/AdhereMujocoHardware tick rate (500Hz); inner_decimation_/
  // outer_decimation_ are read as ROS parameters in on_configure() (defaults 2 and 20 -> 250Hz
  // inner / 25Hz outer). decimator_ is reconstructed from them and reset to tick 0 in
  // on_activate() so a deactivate/reactivate cycle always restarts phase-aligned.
  int cm_update_rate_ = 500;
  int inner_decimation_ = 2;
  int outer_decimation_ = 20;
  std::shared_ptr<adhere_controller_common::MultiRateDecimator> decimator_;
  // Constant inner-loop period (inner_decimation_ / cm_update_rate_ seconds) passed to every
  // FSMState::run() call -- never measured from wall-clock deltas, so it carries zero scheduling
  // jitter regardless of executor/OS timing noise.
  rclcpp::Duration inner_period_ = rclcpp::Duration::from_seconds(0.004);
};

}  // namespace adhere_kamr_controller

#endif  // ADHERE_KAMR_CONTROLLER__ADHERE_KAMR_CONTROLLER_HPP_
