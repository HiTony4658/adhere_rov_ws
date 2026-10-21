#ifndef ADHERE_CONTROLLER__ADHERE_CONTROLLER_HPP_
#define ADHERE_CONTROLLER__ADHERE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "controller_interface/controller_interface.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "adhere_control_msgs/msg/adhere_target_command.hpp"
#include "adhere_control_msgs/msg/controller_state.hpp"
#include "adhere_control_msgs/msg/fsm_command.hpp"
#include "adhere_control_msgs/msg/runtime_diagnostics.hpp"
#include "adhere_control_msgs/msg/setpoint_command.hpp"
#include "adhere_control_msgs/msg/teleop_command.hpp"
#include "adhere_controller/common/ControllerMetrics.h"
#include "adhere_controller/common/enumClass.h"
#include "adhere_controller/common/LoopProfiler.h"
#include "adhere_controller/common/MultiRateDecimator.h"
#include "adhere_controller/algorithms/baseline_mppi/BaselineMppiConfig.h"
#include "adhere_controller/algorithms/baseline_mppi/BaselineMppiController.h"
#include "adhere_controller/CtrlInterfaces.h"
#include "adhere_controller/dynamics/ModelParams.h"
#include "adhere_controller/estimator/RovStateEstimator.h"
#include "adhere_controller/FSM/FSMState.h"
#include "adhere_controller/FSM/StateAdherePid.h"
#include "adhere_controller/FSM/StateAdhereSmc.h"
#include "adhere_controller/FSM/StateBaselineMppi.h"
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
// machine; Phase 3.1 only wires IDLE_PASSIVE_NONE and HOLD_PID_DEPTH.
class AdhereController : public controller_interface::ControllerInterface
{
public:
  AdhereController() = default;

  // Defensive join of the async outer-loop worker (CLAUDE.md Phase 3.6 Stage 2.2) in case the
  // controller is destroyed without on_deactivate() having run first.
  ~AdhereController() override
  {
    outer_worker_running_.store(false);
    outer_worker_cv_.notify_all();
    if (outer_worker_thread_.joinable()) {
      outer_worker_thread_.join();
    }
  }

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
    std::shared_ptr<StateAdherePid> adhere_pid;
    std::shared_ptr<StateAdhereSmc> adhere_smc;
    std::shared_ptr<StateBaselineMppi> baseline_mppi;
  };

  std::shared_ptr<FSMState> getNextState(
    FSMStateName state_name) const;

  // Builds and publishes RuntimeDiagnostics + the RViz2 HUD marker from the current profiler
  // state (CLAUDE.md Phase 3.6 Stage 1.2/3.2). Called from update() at a fixed 10Hz.
  void publishRuntimeDiagnostics(const rclcpp::Time & time);

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
  rclcpp::Subscription<adhere_control_msgs::msg::AdhereTargetCommand>::SharedPtr
    adhere_target_command_subscription_;
  // Previously-active resolved target, tracked on the executor thread only (the subscription
  // callback), used purely to decide whether a new command crosses adhere_stage.retarget_
  // threshold -- never read by the 500Hz thread (ctrl_interfaces_.adhere_target_ is the only
  // cross-thread field).
  double last_adhere_target_y_ = 0.0;
  double last_adhere_target_depth_m_ = 1.0;
  // 500Hz-thread-side counter for detecting a new USE_CURRENT request (see AdhereTargetState's
  // command_seq doc comment in CtrlInterfaces.h).
  uint64_t last_adhere_command_seq_processed_ = 0;
  // Phase 3.6: replaces FsmState.msg/fsm_state_publisher_ -- 100% Sim-to-Real pure, no ground
  // truth field at all (that now only exists on adhere_mujoco_node's /rov/ground_truth_odom).
  rclcpp::Publisher<adhere_control_msgs::msg::ControllerState>::SharedPtr controller_state_publisher_;

  // Phase 3.6 Stage 1.2: real-time frequency/latency observability, published at 10Hz
  // independently of the inner/outer decimation schedule (diag_decimation_ ticks, see
  // on_configure()).
  rclcpp::Publisher<adhere_control_msgs::msg::RuntimeDiagnostics>::SharedPtr
    runtime_diagnostics_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr diagnostics_hud_publisher_;
  int diag_decimation_ = 50;  // cm_update_rate_/10 -> 10Hz, recomputed in on_configure()
  uint64_t diag_tick_count_ = 0;

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
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    diag_state_interface_;  // size 2: [hw_loop_hz, hw_step_exec_ms] (Phase 3.6 Stage 1.2)

  // RovStateEstimator instance driven at 500Hz by update(), ahead of the inner-loop gate, only
  // when model_params_.estimation_mode == "sensor_fusion".
  RovStateEstimator estimator_;
  bool estimator_initialized_ = false;

  // Phase 3.7 output safety gate (common/OutputSafety.h): counts how many inner-loop ticks had
  // to be zeroed because of a NaN/Inf thruster command. Logged (throttled) from update(), not
  // published on any topic yet -- see CLAUDE.md Phase 3.7.
  uint64_t output_safety_violation_count_ = 0;

  // Real-time instrumentation (CLAUDE.md Phase 3.6 Stage 1.2/3.2): budgets match
  // RuntimeDiagnostics.msg's documented inner_loop_budget_ms/outer_loop_budget_ms (4.0ms for
  // 250Hz, 40.0ms for 25Hz); estimator has no documented budget (no overrun field for it in the
  // message), so its profiler is budget-less (overrun counting disabled). Sensor profilers only
  // ever call tickMarker() (never beginExec()/endExec()), since "how often does a new sample
  // arrive" -- not "how long did processing take" -- is what RuntimeDiagnostics wants from them.
  LoopProfiler estimator_profiler_;
  LoopProfiler inner_profiler_{4.0};
  LoopProfiler outer_profiler_{40.0};
  LoopProfiler sensor_imu_profiler_;
  LoopProfiler sensor_loadcell_profiler_;
  LoopProfiler sensor_depth_profiler_;
  LoopProfiler sensor_dvl_profiler_;
  // Edge-detection state for the sample-and-hold noisy depth/DVL channels (mirrors
  // RovStateEstimator's own "has this sample actually changed" pattern) -- sensor_*_profiler_
  // only calls tickMarker() on a genuine new sample, not every 500Hz tick.
  bool have_last_depth_diag_ = false;
  double last_depth_diag_ = 0.0;
  bool have_last_dvl_diag_ = false;
  double last_dvl_diag_ = 0.0;

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

  // Pre-MPPI contact-force distribution & Rhombic Kern matrices (CLAUDE.md Phase 3.6 Stage
  // 2.2), precomputed once in on_activate() from model_params_ -- same lifecycle point every
  // FSMState builds its own B/B^dagger. Not yet consumed by any live constraint (that lands
  // with Phase 4's 12D OSQP); used here only to estimate ControllerState::contact_forces/
  // kern_margin from the commanded thruster forces.
  Mat48 a_contact_ = Mat48::Zero();
  Mat48 c_kern_ = Mat48::Zero();
  double kern_l_y_ = 1.0;
  double kern_l_z_ = 1.0;
  // Phase 4.1: cached alongside a_contact_/c_kern_ above (same on_activate() precompute site) so
  // BaselineMppiController::init() doesn't have to rebuild B/B^dagger itself.
  Mat68 b_ = Mat68::Zero();
  Mat86 b_pinv_ = Mat86::Zero();

  // Phase 4.1: pure-CUDA outer-loop-only Baseline MPPI engine (algorithms/baseline_mppi/), owned
  // here (not by StateBaselineMppi) because outerWorkerLoop() below -- running on its own thread,
  // independent of which FSMState is currently active -- is what actually calls solve().
  // baseline_mppi_config_ is loaded once in on_configure(), same lifecycle point as model_params_.
  baseline_mppi::BaselineMppiConfig baseline_mppi_config_;
  baseline_mppi::BaselineMppiController baseline_mppi_;

  // Phase 3.8 centralized efficiency/chattering/safety telemetry: updated at the 250Hz
  // inner-loop rate from the actually-sent thruster commands, published on ControllerState.msg
  // at the outer-loop rate; reset on every FSM state transition.
  ControllerMetricsAccumulator controller_metrics_;
  ControllerMetricsSample latest_metrics_;

  // Async outer-loop worker skeleton (CLAUDE.md Phase 3.6 Stage 2.2/Sec.5): non-blocking
  // hand-off to a dedicated thread, so the 500Hz update() never waits on outer-loop compute --
  // exactly the architecture the async CUDA MPPI rollout (Phase 4) will reuse, just with a
  // trivial pass-through "solve" for now. Double-buffered input/output guarded by a single
  // mutex + condition_variable (not realtime_tools::RealtimeBuffer -- that lock-free SPSC
  // structure is the right choice once the worker runs at a real, bounded cadence under the
  // outer-loop budget; a plain mutex is simpler and sufficient while the worker does ~nothing).
  struct OuterLoopInput
  {
    Vec3 pos_w = Vec3::Zero();
    Quat quat_w = Quat(1.0, 0.0, 0.0, 0.0);
    Vec3 vel_body = Vec3::Zero();
    Vec3 omega_body = Vec3::Zero();
    Vec3 target_pos = Vec3::Zero();
    Quat target_quat = Quat(1.0, 0.0, 0.0, 0.0);
    uint64_t seq = 0;
    // Phase 4.1: set from current_state_->state_name at the same 500Hz-thread snapshot site
    // (update()'s tick.outer_due block) that fills the fields above -- tells outerWorkerLoop()
    // whether to actually run baseline_mppi_.solve() this cycle, or keep the original trivial
    // pass-through (e.g. while IDLE_PASSIVE_NONE/HOLD_PID_6DOF/ADHERE_* are active).
    bool run_baseline_mppi = false;
  };
  struct OuterLoopOutput
  {
    uint64_t seq = 0;  // echoes OuterLoopInput::seq this output was computed from
    // Phase 4.1: BaselineMppiController::solve()'s result, copied into
    // ctrl_interfaces_.outer_loop_cmd_ every 500Hz tick (see update()) for StateBaselineMppi's
    // ZOH run() to read lock-free.
    std::array<double, 8> u_star{};
    bool valid = false;
    double solve_latency_ms = 0.0;
  };

  void outerWorkerLoop();

  std::thread outer_worker_thread_;
  std::mutex outer_worker_mutex_;
  std::condition_variable outer_worker_cv_;
  OuterLoopInput outer_worker_input_;
  OuterLoopOutput outer_worker_output_;
  bool outer_worker_has_work_ = false;
  std::atomic<bool> outer_worker_running_{false};
  uint64_t outer_worker_seq_ = 0;

  // Phase 4.1: 500Hz-thread-owned copy of OuterLoopOutput::solve_latency_ms (read under
  // outer_worker_mutex_ alongside the ctrl_interfaces_.outer_loop_cmd_ copy in update(), never
  // touched by outerWorkerLoop() itself) -- feeds RuntimeDiagnostics::outer_planner_solve_ms.
  double baseline_mppi_solve_ms_ = 0.0;
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__ADHERE_CONTROLLER_HPP_
