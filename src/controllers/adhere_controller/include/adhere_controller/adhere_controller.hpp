#ifndef ADHERE_CONTROLLER__ADHERE_CONTROLLER_HPP_
#define ADHERE_CONTROLLER__ADHERE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "controller_interface/controller_interface.hpp"

#include "adhere_control_msgs/msg/controller_state.hpp"
#include "adhere_control_msgs/msg/fsm_command.hpp"
#include "adhere_control_msgs/msg/setpoint_command.hpp"
#include "adhere_control_msgs/msg/teleop_command.hpp"
#include "adhere_controller/common/enumClass.h"
#include "adhere_controller/common/ModelParams.h"
#include "adhere_controller/common/MultiRateDecimator.h"
#include "adhere_controller/CtrlInterfaces.h"
#include "adhere_controller/estimator/RovStateEstimator.h"
#include "adhere_controller/FSM/FSMState.h"
#include "adhere_controller/FSM/StateDepthHold.h"
#include "adhere_controller/FSM/StateOpenLoop.h"
#include "adhere_controller/FSM/StatePassive.h"
#include "adhere_controller/FSM/StatePidHold.h"

namespace adhere_controller
{

// AdhereController: the ROS 2 Foxy controller_interface::ControllerInterface plugin that
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
class AdhereController : public controller_interface::ControllerInterface
{
public:
  AdhereController() = default;

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
    std::shared_ptr<StatePassive> passive;
    std::shared_ptr<StateDepthHold> depth_hold;
    std::shared_ptr<StateOpenLoop> open_loop;
    std::shared_ptr<StatePidHold> pid_hold;
  };

  std::shared_ptr<FSMState> getNextState(
    FSMStateName state_name) const;

  CtrlInterfaces ctrl_interfaces_;
  // Runtime-loaded model-decoupled parameters (CLAUDE.md Phase 3.3 Sec.3), populated once in
  // on_configure() and passed by const reference into every FSMState constructed in
  // on_activate(). FSMState instances hold onto the SAME ModelParams for their whole lifetime
  // (never reloaded without a deactivate/configure/activate cycle).
  ModelParams model_params_;

  FSMMode mode_ = FSMMode::NORMAL;
  FSMStateList state_list_;
  std::shared_ptr<FSMState> current_state_;
  std::shared_ptr<FSMState> next_state_;
  FSMStateName next_state_name_ =
    FSMStateName::INVALID;

  rclcpp::Subscription<adhere_control_msgs::msg::FsmCommand>::SharedPtr fsm_command_subscription_;
  rclcpp::Subscription<adhere_control_msgs::msg::SetpointCommand>::SharedPtr
    setpoint_command_subscription_;
  rclcpp::Subscription<adhere_control_msgs::msg::TeleopCommand>::SharedPtr
    teleop_command_subscription_;
  // Phase 3.6: replaces FsmState.msg/fsm_state_publisher_ -- 100% Sim-to-Real pure, no ground
  // truth field at all (that now only exists on adhere_mujoco_node's /rov/ground_truth_odom).
  rclcpp::Publisher<adhere_control_msgs::msg::ControllerState>::SharedPtr controller_state_publisher_;

  // Phase 3.5 strict ground-truth isolation (CLAUDE.md Stage 2.3): these raw hardware state
  // interfaces are PRIVATE to AdhereController -- never exposed via CtrlInterfaces, never
  // reachable from any FSMState. gt_* hold noise-free ground truth (used only to populate
  // FsmState::ground_truth_pose for offline evaluation, and to populate
  // ctrl_interfaces_.active_state directly in "ground_truth" mode); noisy_* hold the realistic
  // multi-rate sensors consumed exclusively by estimator_ in "sensor_fusion" mode.
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    gt_imu_state_interface_;          // size 10
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    gt_odom_state_interface_;         // size 10
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    gt_pad_contact_state_interface_;  // size 8
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    noisy_imu_state_interface_;       // size 10
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    noisy_depth_state_interface_;     // size 1
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    noisy_dvl_state_interface_;       // size 4
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    noisy_loadcell_state_interface_;  // size 4

  // RovStateEstimator instance driven at 500Hz by update(), ahead of the inner-loop gate, only
  // when model_params_.estimation_mode == "sensor_fusion".
  RovStateEstimator estimator_;
  bool estimator_initialized_ = false;

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
  std::shared_ptr<MultiRateDecimator> decimator_;
  // Constant inner-loop period (inner_decimation_ / cm_update_rate_ seconds) passed to every
  // FSMState::run() call -- never measured from wall-clock deltas, so it carries zero scheduling
  // jitter regardless of executor/OS timing noise.
  rclcpp::Duration inner_period_ = rclcpp::Duration::from_seconds(0.004);
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__ADHERE_CONTROLLER_HPP_
