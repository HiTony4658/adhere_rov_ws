#include "adhere_controller/adhere_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "adhere_controller/allocation/ThrusterAllocator.h"
#include "adhere_controller/common/mathTools.h"
#include "adhere_controller/common/OutputSafety.h"

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

  // Phase 3.6 Stage 1.2: runtime diagnostics mirrors from AdhereMujocoHardware.
  names.emplace_back("diagnostics/hw_loop_hz");
  names.emplace_back("diagnostics/hw_step_exec_ms");

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

  // This phase: new 5th topic, strictly scoped to live-mutating the ADHERE_* wall target
  // (CLAUDE.md Sec.3.4 amendment) -- resolves TAG_PRESET against model_params_.adhere_wall_tags,
  // CUSTOM_* against the message's own fields, USE_CURRENT against the live active_state
  // estimate, and writes the result into ctrl_interfaces_.adhere_target_ for every ADHERE_*
  // FSMState to read lock-free.
  adhere_target_command_subscription_ =
    get_node()->create_subscription<adhere_control_msgs::msg::AdhereTargetCommand>(
      "/rov/adhere_target_cmd", 10,
      [this](const adhere_control_msgs::msg::AdhereTargetCommand::SharedPtr msg) {
        // USE_CURRENT needs ActiveState, which is only ever safely read on the 500Hz thread
        // (CLAUDE.md Phase 3.5's ground-truth isolation discipline) -- deferred to
        // AdhereController::update() via use_current_requested/seq instead of resolved
        // here. TAG_PRESET/CUSTOM_* are pure lookups against model_params_/the message itself,
        // so they can be (and are) resolved immediately on this executor thread.
        if (msg->command_type == adhere_control_msgs::msg::AdhereTargetCommand::USE_CURRENT) {
          ctrl_interfaces_.adhere_target_.use_current_requested.store(true);
          ctrl_interfaces_.adhere_target_.seq.fetch_add(1);
          return;
        }
        double new_y = last_adhere_target_y_;
        double new_depth_m = last_adhere_target_depth_m_;
        switch (msg->command_type) {
          case adhere_control_msgs::msg::AdhereTargetCommand::TAG_PRESET:
            if (msg->tag_id < model_params_.adhere_wall_tags.size()) {
              new_y = model_params_.adhere_wall_tags[msg->tag_id].y;
              new_depth_m = model_params_.adhere_wall_tags[msg->tag_id].depth_m;
            }
            break;
          case adhere_control_msgs::msg::AdhereTargetCommand::CUSTOM_Y:
            new_y = msg->target_y;
            break;
          case adhere_control_msgs::msg::AdhereTargetCommand::CUSTOM_DEPTH:
            new_depth_m = msg->target_depth_m;
            break;
          case adhere_control_msgs::msg::AdhereTargetCommand::CUSTOM_Y_DEPTH:
            new_y = msg->target_y;
            new_depth_m = msg->target_depth_m;
            break;
          default:
            return;
        }
        applyResolvedAdhereTarget(new_y, new_depth_m);
      });

  // Phase 5.0 Stage 6: measured wall-tag detections (adhere_apriltag_detector, sim or real
  // apriltag_ros -- identical topic contract) feed the SAME resolved-target bridge/threshold
  // logic as /rov/adhere_target_cmd above, kept on a strictly separate topic since this is a
  // MEASUREMENT, not a human/preset command (CLAUDE.md Sec.9's TagDetection doc comment). A
  // fresh valid detection supersedes whatever is currently active (preset or an earlier
  // detection) -- last-writer-wins on this single shared bridge is exactly the "a received tag
  // governs over a later disagreeing preset until explicitly re-presetted" contract: either
  // channel can re-assert at any time, and the detector keeps re-publishing while the ROV stays
  // in range, so a stale preset never silently lingers near an instrumented station.
  tag_detection_subscription_ =
    get_node()->create_subscription<adhere_control_msgs::msg::TagDetection>(
      "/rov/tag_detection", 10,
      [this](const adhere_control_msgs::msg::TagDetection::SharedPtr msg) {
        if (!msg->valid) {
          return;
        }
        applyResolvedAdhereTarget(msg->target_y, msg->target_depth_m);
      });

  controller_state_publisher_ = get_node()->create_publisher<adhere_control_msgs::msg::ControllerState>(
    "/rov/controller_state", 10);

  // CLAUDE.md Phase 5.0 Stage 7: offline-evaluation/benchmark telemetry, split off
  // ControllerState -- only benchmark_evaluator_node consumes this.
  benchmark_metrics_publisher_ =
    get_node()->create_publisher<adhere_control_msgs::msg::BenchmarkMetrics>(
      "/rov/benchmark_metrics", 10);

  runtime_diagnostics_publisher_ =
    get_node()->create_publisher<adhere_control_msgs::msg::RuntimeDiagnostics>(
      "/rov/runtime_diagnostics", 10);
  diagnostics_hud_publisher_ = get_node()->create_publisher<visualization_msgs::msg::Marker>(
    "/rov/diagnostics_hud_marker", 10);

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
  // Phase 5.0 Stage 4: (re)construct the estimator with the just-loaded gains -- same lifecycle
  // point model_params_ itself is populated, so a reconfigure always picks up a fresh YAML read.
  estimator_ = RovStateEstimator(model_params_.estimator);
  // Phase 4.1: algorithm-scoped config, loaded independently of ModelParams (CLAUDE.md Sec.6 of
  // this phase's task spec) -- same lifecycle point, own parser (BaselineMppiConfig.cpp).
  baseline_mppi_config_ = baseline_mppi::BaselineMppiConfig::loadFromNode(get_node());

  // Named-channel multi-rate registry (CLAUDE.md Phase 5.0 Stage 3): every rate this controller
  // runs at is one registered channel, replacing the former separate decimator_/diag_decimation_/
  // diag_tick_count_ mechanisms. "diag" is fixed at 10Hz, independent of inner_decimation_/
  // outer_decimation_ (so changing those never silently changes the diagnostics rate too).
  // "estimator" is registered as decimation=1 purely to document its every-tick cadence
  // explicitly -- it is still gated on model_params_.estimation_mode=="sensor_fusion" in
  // update(), not on this channel's due-ness, since it legitimately does not run at all in
  // "ground_truth" mode.
  decimator_ = std::make_shared<MultiRateDecimator>(cm_update_rate_);
  inner_channel_ = decimator_->addChannel("inner", inner_decimation_);
  outer_channel_ = decimator_->addChannel("outer", outer_decimation_);
  diag_channel_ = decimator_->addChannel("diag", std::max(1, cm_update_rate_ / 10));
  estimator_channel_ = decimator_->addChannel("estimator", 1);
  inner_period_ = rclcpp::Duration::from_seconds(decimator_->periodSeconds(inner_channel_));

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
  diag_state_interface_.clear();
  estimator_initialized_ = false;
  estimator_profiler_.reset();
  inner_profiler_.reset();
  outer_profiler_.reset();
  sensor_imu_profiler_.reset();
  sensor_loadcell_profiler_.reset();
  sensor_depth_profiler_.reset();
  sensor_dvl_profiler_.reset();
  have_last_depth_diag_ = false;
  have_last_dvl_diag_ = false;

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

  findAndAppend(state_interfaces_, "diagnostics/hw_loop_hz", diag_state_interface_);
  findAndAppend(state_interfaces_, "diagnostics/hw_step_exec_ms", diag_state_interface_);

  // Pre-MPPI A_contact/C_kern precompute (CLAUDE.md Phase 3.6 Stage 2.2): same lifecycle point
  // every FSMState builds its own B/B^dagger, from the same model_params_ geometry.
  {
    b_ = buildThrusterAllocationMatrix(model_params_.thruster_pos, model_params_.thruster_dir);
    b_pinv_ = buildThrusterAllocationPseudoInverse(b_);
    const Mat34 g = buildPadNormalMap(model_params_.pad_pos);
    const Mat43 g_pinv = buildPadNormalMapPseudoInverse(g);
    a_contact_ = buildContactMap(g_pinv, b_);
    const auto spans = padHalfSpans(model_params_.pad_pos);
    kern_l_y_ = spans.first;
    kern_l_z_ = spans.second;
    c_kern_ = buildKernConstraintMatrix(b_, kern_l_y_, kern_l_z_, model_params_.kern_eps_margin);
  }

  // Phase 4.1: one-time GPU buffer allocation for the Baseline MPPI engine, same lifecycle point
  // as the A_contact/C_kern precompute above (on_activate(), using the SAME B/B^dagger).
  baseline_mppi_.init(model_params_, b_, b_pinv_, baseline_mppi_config_);

  // This phase: seed the live wall-target bridge from the static config default --
  // /rov/adhere_target_cmd (subscribed in on_configure()) only ever mutates it from here
  // onward, so behavior is unchanged from Phase 4.3 until the first live command arrives.
  ctrl_interfaces_.adhere_target_.target_y.store(model_params_.adhere_stage.target_y);
  ctrl_interfaces_.adhere_target_.target_depth_m.store(model_params_.adhere_stage.target_depth_m);
  ctrl_interfaces_.adhere_target_.valid.store(true);
  ctrl_interfaces_.adhere_target_.changed_pending.store(false);
  last_adhere_target_y_ = model_params_.adhere_stage.target_y;
  last_adhere_target_depth_m_ = model_params_.adhere_stage.target_depth_m;

  // Async outer-loop worker skeleton (CLAUDE.md Phase 3.6 Stage 2.2): started here so a
  // deactivate/reactivate cycle always gets a fresh thread, joined in on_deactivate() below.
  outer_worker_seq_ = 0;
  outer_worker_has_work_ = false;
  outer_worker_running_.store(true);
  baseline_mppi_solve_ms_ = 0.0;
  ctrl_interfaces_.outer_loop_cmd_.valid.store(false);
  outer_worker_thread_ = std::thread(&AdhereController::outerWorkerLoop, this);

  // Single FSM instance table (CLAUDE.md Phase 5.0 Stage 2.1), order matches
  // common/FsmRegistry.h's kFsmRegistry (not load-bearing, just conventional).
  fsm_states_ = {
    {FSMStateName::IDLE_PASSIVE_NONE, std::make_shared<StatePassive>(ctrl_interfaces_, model_params_)},
    {FSMStateName::TELEOP_OPENLOOP_6DOF,
      std::make_shared<StateOpenLoop>(ctrl_interfaces_, model_params_)},
    {FSMStateName::HOLD_PID_6DOF, std::make_shared<StatePidHold>(ctrl_interfaces_, model_params_)},
    {FSMStateName::ADHERE_PID_WBC,
      std::make_shared<StateAdherePid>(ctrl_interfaces_, model_params_)},
    {FSMStateName::ADHERE_SMC_WBC,
      std::make_shared<StateAdhereSmc>(ctrl_interfaces_, model_params_)},
    {FSMStateName::ADHERE_MPPI_BASELINE,
      std::make_shared<StateBaselineMppi>(ctrl_interfaces_, model_params_, baseline_mppi_)},
  };

  // Default-boots in IDLE_PASSIVE_NONE (CLAUDE.md Phase 3.1 Stage 2.3): launching mujoco_sim.launch.py
  // immediately brings up the FSM in the safe zero-thrust state.
  current_state_ = getNextState(FSMStateName::IDLE_PASSIVE_NONE);
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
  // Stop & join the async outer-loop worker (CLAUDE.md Phase 3.6 Stage 2.2) before releasing
  // interfaces, so it never touches ctrl_interfaces_ after this point.
  outer_worker_running_.store(false);
  outer_worker_cv_.notify_all();
  if (outer_worker_thread_.joinable()) {
    outer_worker_thread_.join();
  }

  release_interfaces();
  ctrl_interfaces_.clear();
  gt_imu_state_interface_.clear();
  gt_odom_state_interface_.clear();
  gt_pad_contact_state_interface_.clear();
  noisy_imu_state_interface_.clear();
  noisy_depth_state_interface_.clear();
  noisy_dvl_state_interface_.clear();
  noisy_loadcell_state_interface_.clear();
  diag_state_interface_.clear();
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
  // the decimator reports, per registered channel, which of the uniform schedules are due on
  // this tick (CLAUDE.md Phase 5.0 Stage 3).
  const auto due = decimator_->tick();
  const bool inner_due = due[inner_channel_];
  const bool outer_due = due[outer_channel_];
  const bool diag_due = due[diag_channel_];

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

  // Sensor sample-rate instrumentation (CLAUDE.md Phase 3.6 Stage 1.2/3.2): AdhereMujocoHardware
  // populates the noisy sensor blocks every 500Hz tick regardless of estimation_mode, so these
  // measure the true hardware sampling cadence, not just "whenever sensor_fusion consumes them".
  // IMU/loadcell update every tick (500Hz); depth/DVL are sample-and-hold, so tickMarker() only
  // fires on a genuine new sample (edge-detected exactly like RovStateEstimator's own
  // correctDepth()/correctDvl() pattern).
  sensor_imu_profiler_.tickMarker();
  sensor_loadcell_profiler_.tickMarker();
  const double depth_sample = noisy_depth_state_interface_[0].get().get_value();
  if (!have_last_depth_diag_ || depth_sample != last_depth_diag_) {
    sensor_depth_profiler_.tickMarker();
    have_last_depth_diag_ = true;
    last_depth_diag_ = depth_sample;
  }
  const double dvl_sample = noisy_dvl_state_interface_[0].get().get_value();
  if (!have_last_dvl_diag_ || dvl_sample != last_dvl_diag_) {
    sensor_dvl_profiler_.tickMarker();
    have_last_dvl_diag_ = true;
    last_dvl_diag_ = dvl_sample;
  }

  if (model_params_.estimation_mode == "sensor_fusion") {
    estimator_profiler_.tickMarker();
    estimator_profiler_.beginExec();
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

    // Phase 3.7 contact-aided wall-constraint pseudo-measurement (CLAUDE.md Stage 1.4):
    // arrests normal-axis drift once all 4 raw pad load cells confirm stable wall contact.
    std::array<double, 4> pad_force_raw;
    for (size_t i = 0; i < kPadNames.size(); ++i) {
      pad_force_raw[i] = noisy_loadcell_state_interface_[i].get().get_value();
    }
    estimator_.correctContact(
      pad_force_raw, model_params_.contact_force_thresh, model_params_.workspace_x_min);

    const Vec3 est_pos = estimator_.positionWorld();
    const Vec3 est_vel = estimator_.velocityBody();
    const Vec3 est_omega = estimator_.angularVelocityBody();
    const Quat est_quat = estimator_.orientation();
    for (int i = 0; i < 3; ++i) {
      ctrl_interfaces_.active_state.pos_w[i] = est_pos(i);
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
    estimator_profiler_.endExec();
  } else {
    estimator_initialized_ = false;
    for (int i = 0; i < 3; ++i) {
      ctrl_interfaces_.active_state.pos_w[i] = gt_pos(i);
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

  // This phase: resolve a deferred USE_CURRENT wall-retarget request now that active_state is
  // fresh for this tick (see AdhereTargetState's doc comment for why this can't be resolved on
  // the executor thread).
  {
    const uint64_t seq = ctrl_interfaces_.adhere_target_.seq.load();
    if (seq != last_adhere_command_seq_processed_ &&
        ctrl_interfaces_.adhere_target_.use_current_requested.load())
    {
      const double new_y = ctrl_interfaces_.active_state.pos_w[1];
      const double new_depth_m = -ctrl_interfaces_.active_state.pos_w[2];
      const bool crossed_threshold =
        std::abs(new_y - ctrl_interfaces_.adhere_target_.target_y.load()) >
          model_params_.adhere_stage.retarget_threshold ||
        std::abs(new_depth_m - ctrl_interfaces_.adhere_target_.target_depth_m.load()) >
          model_params_.adhere_stage.retarget_threshold;
      ctrl_interfaces_.adhere_target_.target_y.store(new_y);
      ctrl_interfaces_.adhere_target_.target_depth_m.store(new_depth_m);
      if (crossed_threshold) {
        ctrl_interfaces_.adhere_target_.changed_pending.store(true);
      }
      ctrl_interfaces_.adhere_target_.use_current_requested.store(false);
      last_adhere_command_seq_processed_ = seq;
    }
  }

  if (inner_due) {
    inner_profiler_.tickMarker();
    inner_profiler_.beginExec();

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
      // Centralized output safety gate (CLAUDE.md Phase 3.7): the single place
      // ci.pending_thruster_cmd[0..7] (written by FSMState::run() above) is sanitized and
      // actually reaches hardware -- never a direct FSMState -> hw_commands_ write anymore.
      const uint64_t violations_before = output_safety_violation_count_;
      sanitizeAndWriteThrusterCommands(
        ctrl_interfaces_, model_params_, output_safety_violation_count_);
      if (output_safety_violation_count_ != violations_before) {
        RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Output safety gate: NaN/Inf thruster command detected, zeroing all 8 thrusters "
          "(total violations: %lu)",
          static_cast<unsigned long>(output_safety_violation_count_));
      }

      // Centralized efficiency/chattering/safety telemetry (CLAUDE.md Phase 3.8): computed from
      // the actually-sent (sanitized+saturated) thruster values, read back via the command
      // interface itself (not ci.pending_thruster_cmd, which is pre-clamp) immediately after the
      // output safety gate above.
      std::array<double, 8> u_sent{};
      Vec8 u_sent_vec;
      for (int i = 0; i < 8; ++i) {
        u_sent[i] = ctrl_interfaces_.thruster_command_interface_[i].get().get_value();
        u_sent_vec(i) = u_sent[i];
      }
      const Vec4 contact_forces_est_vec = a_contact_ * u_sent_vec;
      const std::array<double, 4> contact_forces_est = {
        contact_forces_est_vec(0), contact_forces_est_vec(1), contact_forces_est_vec(2),
        contact_forces_est_vec(3)};
      latest_metrics_ = controller_metrics_.update(
        u_sent, contact_forces_est, model_params_, inner_period_.seconds());

      next_state_name_ = current_state_->checkChange();
      if (next_state_name_ != current_state_->state_name) {
        mode_ = FSMMode::CHANGE;
        next_state_ = getNextState(next_state_name_);
      }
    } else {
      current_state_->exit();
      current_state_ = next_state_;
      current_state_->enter();
      // Reset cumulative energy/impulse integrators on every FSM state transition (CLAUDE.md
      // Phase 3.8), so each state's telemetry reflects only the time spent in that state.
      controller_metrics_.reset();
      mode_ = FSMMode::NORMAL;
    }

    inner_profiler_.endExec();
  }

  if (outer_due) {
    outer_profiler_.tickMarker();
    outer_profiler_.beginExec();

    // Live terminal feedback (CLAUDE.md Phase 3.6's ControllerState.msg, 100% Sim-to-Real pure
    // -- no ground truth here at all): published here (not gated on inner_due) so
    // fsm_console_node sees live estimated_pose/target_pose at the outer-loop rate regardless
    // of which inner-loop state is active.
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
    controller_state_msg.target_valid = current_state_->targetValid();
    controller_state_msg.adhere_stage = current_state_->adhereStageName();

    const auto & active = ctrl_interfaces_.active_state;

    // Async outer-loop worker hand-off (CLAUDE.md Phase 3.6 Stage 2.2): non-blocking snapshot
    // of the current state + target into the shared input buffer, waking outerWorkerLoop().
    // The real async CUDA MPPI rollout (qpos, qvel, F_x_extra_preload -> w_cmd) is a Phase 4
    // milestone -- the worker below only does a trivial pass-through "solve" for now, but the
    // threading/hand-off architecture itself is already wired and running at the uniform 25Hz
    // outer-loop rate (see adhere_controller's test_multirate_decimator).
    {
      std::lock_guard<std::mutex> lock(outer_worker_mutex_);
      outer_worker_input_.pos_w = Vec3(active.pos_w[0], active.pos_w[1], active.pos_w[2]);
      outer_worker_input_.quat_w =
        Quat(active.quat_w[0], active.quat_w[1], active.quat_w[2], active.quat_w[3]);
      outer_worker_input_.vel_body =
        Vec3(active.vel_body[0], active.vel_body[1], active.vel_body[2]);
      outer_worker_input_.omega_body =
        Vec3(active.omega_body[0], active.omega_body[1], active.omega_body[2]);
      outer_worker_input_.target_pos = target.pos;
      outer_worker_input_.target_quat = target.quat;
      outer_worker_input_.seq = ++outer_worker_seq_;
      // Phase 4.1: only the async worker's Baseline MPPI branch actually spends GPU time -- every
      // other FSM state keeps the original trivial pass-through (see outerWorkerLoop()).
      outer_worker_input_.run_baseline_mppi =
        current_state_->state_name == FSMStateName::ADHERE_MPPI_BASELINE;
      outer_worker_has_work_ = true;
    }
    outer_worker_cv_.notify_one();

    // Phase 4.1: copies the worker's latest output into ctrl_interfaces_.outer_loop_cmd_ (lock-free
    // bridge StateBaselineMppi::run() reads, see CtrlInterfaces.h) and into the 500Hz-thread-owned
    // baseline_mppi_solve_ms_ (for publishRuntimeDiagnostics() below) -- cheap mutex-guarded copy,
    // every outer_due tick, same mutex the snapshot hand-off above already uses.
    {
      std::lock_guard<std::mutex> lock(outer_worker_mutex_);
      for (int i = 0; i < 8; ++i) {
        ctrl_interfaces_.outer_loop_cmd_.u_star[i].store(outer_worker_output_.u_star[i]);
      }
      ctrl_interfaces_.outer_loop_cmd_.valid.store(outer_worker_output_.valid);
      baseline_mppi_solve_ms_ = outer_worker_output_.solve_latency_ms;
    }

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
    Vec8 u_cmd;
    for (int i = 0; i < 8; ++i) {
      u_cmd(i) = ctrl_interfaces_.thruster_state_interface_[i].get().get_value();
      controller_state_msg.thruster_cmds[i] = u_cmd(i);
    }

    // contact_forces/kern_margin (CLAUDE.md Sec.4.4/Sec.5, Phase 3.6 Stage 2.2): estimated from
    // the commanded thruster forces via A_contact, NOT read from the real pad-contact sensors --
    // this is the quasi-static prediction the future KAMR-MPPI's OSQP constraint will also use.
    const Vec4 f_contact_est = a_contact_ * u_cmd;
    for (int i = 0; i < 4; ++i) {
      controller_state_msg.contact_forces[i] = f_contact_est(i);
    }
    // Center-of-pressure / K_val is only meaningful while the hull is actually pressing against
    // the wall (sum_f appreciably > 0) -- when it isn't (e.g. HOLD_PID_6DOF floating away from the
    // wall), dividing by a near-zero sum_f would blow p_y/p_z/K_val up into meaningless large
    // values. kern_margin reports the safe default (1.0, "fully inside kern") below that
    // threshold instead of garbage telemetry; kFContactFloor is a noise-floor-scale threshold,
    // not a physical rail limit.
    constexpr double kFContactFloor = 1.0;  // N
    const double sum_f = f_contact_est.sum();
    double k_val = 0.0;
    if (sum_f > kFContactFloor) {
      const double p_y =
        (f_contact_est(0) * model_params_.pad_pos[0].y() +
         f_contact_est(1) * model_params_.pad_pos[1].y() +
         f_contact_est(2) * model_params_.pad_pos[2].y() +
         f_contact_est(3) * model_params_.pad_pos[3].y()) / sum_f;
      const double p_z =
        (f_contact_est(0) * model_params_.pad_pos[0].z() +
         f_contact_est(1) * model_params_.pad_pos[1].z() +
         f_contact_est(2) * model_params_.pad_pos[2].z() +
         f_contact_est(3) * model_params_.pad_pos[3].z()) / sum_f;
      k_val = std::abs(p_y) / kern_l_y_ + std::abs(p_z) / kern_l_z_;
    }
    // CLAUDE.md Phase 5.0 Stage 7: kern_margin moved off ControllerState onto BenchmarkMetrics
    // (published separately at the 10Hz diag rate, see publishRuntimeDiagnostics() call site
    // below) -- cached here since f_contact_est/sum_f are only available in this outer_due
    // block, not the diag_due one.
    latest_kern_margin_ = 1.0 - k_val;

    controller_state_publisher_->publish(controller_state_msg);
    outer_profiler_.endExec();
  }

  // Real-time diagnostics (CLAUDE.md Phase 3.6 Stage 1.2): fixed 10Hz, independent of the
  // inner/outer decimation schedule above (the "diag" channel registered in on_configure()).
  if (diag_due) {
    publishRuntimeDiagnostics(time);

    // CLAUDE.md Phase 5.0 Stage 7: BenchmarkMetrics, same 10Hz "diag" channel as
    // RuntimeDiagnostics above -- latest_metrics_/latest_kern_margin_ are both already updated
    // (at the 250Hz inner rate / outer_due block respectively) by the time this fires.
    adhere_control_msgs::msg::BenchmarkMetrics benchmark_msg;
    benchmark_msg.header.stamp = time;
    if (current_state_) {
      benchmark_msg.current_state = fsmStateNameToInt(current_state_->state_name);
      benchmark_msg.state_name = current_state_->state_name_string;
    }
    benchmark_msg.thrust_l2_norm = latest_metrics_.thrust_l2_norm;
    benchmark_msg.thrust_power_proxy = latest_metrics_.thrust_power_proxy;
    benchmark_msg.thrust_slew_rate_l2 = latest_metrics_.thrust_slew_rate_l2;
    benchmark_msg.cumulative_energy_l2 = latest_metrics_.cumulative_energy_l2;
    benchmark_msg.cumulative_impulse_l1 = latest_metrics_.cumulative_impulse_l1;
    benchmark_msg.saturated_thruster_count = latest_metrics_.saturated_thruster_count;
    benchmark_msg.active_contact_pad_count = latest_metrics_.active_contact_pad_count;
    benchmark_msg.kern_margin = latest_kern_margin_;
    benchmark_metrics_publisher_->publish(benchmark_msg);
  }

  return controller_interface::return_type::OK;
}

void AdhereController::applyResolvedAdhereTarget(double new_y, double new_depth_m)
{
  const bool crossed_threshold =
    std::abs(new_y - last_adhere_target_y_) > model_params_.adhere_stage.retarget_threshold ||
    std::abs(new_depth_m - last_adhere_target_depth_m_) >
      model_params_.adhere_stage.retarget_threshold;
  ctrl_interfaces_.adhere_target_.target_y.store(new_y);
  ctrl_interfaces_.adhere_target_.target_depth_m.store(new_depth_m);
  if (crossed_threshold) {
    ctrl_interfaces_.adhere_target_.changed_pending.store(true);
  }
  last_adhere_target_y_ = new_y;
  last_adhere_target_depth_m_ = new_depth_m;
}

void AdhereController::publishRuntimeDiagnostics(const rclcpp::Time & time)
{
  adhere_control_msgs::msg::RuntimeDiagnostics diag;
  diag.header.stamp = time;

  diag.hw_loop_hz = diag_state_interface_[0].get().get_value();
  diag.sensor_imu_hz = sensor_imu_profiler_.hz();
  diag.sensor_loadcell_hz = sensor_loadcell_profiler_.hz();
  diag.sensor_depth_hz = sensor_depth_profiler_.hz();
  diag.sensor_dvl_hz = sensor_dvl_profiler_.hz();
  diag.estimator_hz = estimator_profiler_.hz();
  diag.inner_loop_hz = inner_profiler_.hz();
  diag.outer_loop_hz = outer_profiler_.hz();
  diag.thruster_cmd_hz = inner_profiler_.hz();  // thruster write() is gated by the same tick

  diag.hw_step_exec_ms = diag_state_interface_[1].get().get_value();
  diag.estimator_exec_ms = estimator_profiler_.latestExecMs();
  diag.estimator_max_ms = estimator_profiler_.maxExecMs();
  diag.inner_loop_exec_ms = inner_profiler_.latestExecMs();
  diag.inner_loop_max_ms = inner_profiler_.maxExecMs();
  diag.inner_loop_budget_ms = inner_profiler_.budgetMs();
  diag.outer_loop_exec_ms = outer_profiler_.latestExecMs();
  diag.outer_loop_max_ms = outer_profiler_.maxExecMs();
  diag.outer_loop_budget_ms = outer_profiler_.budgetMs();

  diag.inner_loop_overruns = inner_profiler_.overruns();
  diag.outer_loop_overruns = outer_profiler_.overruns();

  // Phase 4.1: 0 whenever no GPU outer-loop planner is active (baseline_mppi_solve_ms_ is only
  // ever updated by update()'s outer_due block above when run_baseline_mppi was set).
  diag.outer_planner_solve_ms = baseline_mppi_solve_ms_;

  // timing_healthy (CLAUDE.md Phase 3.6 Stage 1.2): all measured frequencies within 10% of
  // their target AND latest inner/outer exec <= budget. estimator_hz is deliberately excluded
  // from the frequency check -- it legitimately reports 0 whenever estimation_mode ==
  // "ground_truth" (the estimator simply isn't driven), which is not a timing fault.
  const auto withinTenPercent = [](double measured, double target) {
    return target <= 0.0 || std::abs(measured - target) <= 0.10 * target;
  };
  diag.timing_healthy =
    withinTenPercent(diag.hw_loop_hz, 500.0) &&
    withinTenPercent(diag.sensor_imu_hz, 500.0) &&
    withinTenPercent(diag.sensor_loadcell_hz, 500.0) &&
    withinTenPercent(diag.sensor_depth_hz, 50.0) &&
    withinTenPercent(diag.sensor_dvl_hz, 20.0) &&
    withinTenPercent(diag.inner_loop_hz, 250.0) &&
    withinTenPercent(diag.outer_loop_hz, 25.0) &&
    withinTenPercent(diag.thruster_cmd_hz, 250.0) &&
    diag.inner_loop_exec_ms <= diag.inner_loop_budget_ms &&
    diag.outer_loop_exec_ms <= diag.outer_loop_budget_ms;

  runtime_diagnostics_publisher_->publish(diag);

  // RViz2 3D HUD overlay (CLAUDE.md Phase 3.6 Stage 3.2): floats above the ROV, green when
  // healthy, red on any overrun/off-target frequency.
  visualization_msgs::msg::Marker hud;
  hud.header.frame_id = "world";
  hud.header.stamp = time;
  hud.ns = "runtime_diagnostics";
  hud.id = 0;
  hud.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  hud.action = visualization_msgs::msg::Marker::ADD;
  hud.pose.position.x = ctrl_interfaces_.active_state.pos_w[0];
  hud.pose.position.y = ctrl_interfaces_.active_state.pos_w[1];
  hud.pose.position.z = ctrl_interfaces_.active_state.pos_w[2] + 0.6;
  hud.pose.orientation.w = 1.0;
  hud.scale.z = 0.12;
  hud.color.a = 1.0;
  hud.color.r = diag.timing_healthy ? 0.0 : 1.0;
  hud.color.g = diag.timing_healthy ? 1.0 : 0.0;
  hud.color.b = 0.0;
  hud.lifetime = rclcpp::Duration::from_seconds(0.5);

  char buf[512];
  std::snprintf(
    buf, sizeof(buf),
    "HW %.0fHz/%.2fms  IMU %.0fHz  LC %.0fHz  Depth %.0fHz  DVL %.0fHz\n"
    "Est %.0fHz/%.2fms  Inner %.0fHz/%.2fms(ovr %lu)  Outer %.0fHz/%.2fms(ovr %lu)",
    diag.hw_loop_hz, diag.hw_step_exec_ms, diag.sensor_imu_hz, diag.sensor_loadcell_hz,
    diag.sensor_depth_hz, diag.sensor_dvl_hz, diag.estimator_hz, diag.estimator_exec_ms,
    diag.inner_loop_hz, diag.inner_loop_exec_ms,
    static_cast<unsigned long>(diag.inner_loop_overruns), diag.outer_loop_hz,
    diag.outer_loop_exec_ms, static_cast<unsigned long>(diag.outer_loop_overruns));
  hud.text = buf;

  diagnostics_hud_publisher_->publish(hud);
}

void AdhereController::outerWorkerLoop()
{
  // Async outer-loop worker skeleton (CLAUDE.md Phase 3.6 Stage 2.2): blocks on the condition
  // variable between snapshots (zero busy-polling), wakes on every outer_due tick's notify_one()
  // from update(), and currently performs a trivial pass-through "solve" -- Phase 4 replaces
  // the body of this loop with the real async CUDA MPPI rollout, without touching the
  // thread/hand-off lifecycle itself (started in on_activate(), joined in on_deactivate()/~dtor).
  while (outer_worker_running_.load()) {
    OuterLoopInput input;
    {
      std::unique_lock<std::mutex> lock(outer_worker_mutex_);
      outer_worker_cv_.wait(
        lock, [this] { return outer_worker_has_work_ || !outer_worker_running_.load(); });
      if (!outer_worker_running_.load()) {
        break;
      }
      input = outer_worker_input_;
      outer_worker_has_work_ = false;
    }

    // Phase 4.1: real async CUDA MPPI rollout, gated on run_baseline_mppi (set by update() from
    // current_state_->state_name, CLAUDE.md Sec.3.3) -- every other FSM state still gets the
    // original trivial pass-through, since they never read ctrl_interfaces_.outer_loop_cmd_ at
    // all (only StateBaselineMppi::run() does).
    OuterLoopOutput output;
    output.seq = input.seq;
    if (input.run_baseline_mppi) {
      output.u_star = baseline_mppi_.solve(
        input.pos_w, input.quat_w, input.vel_body, input.omega_body, input.target_pos,
        input.target_quat);
      output.valid = true;
      output.solve_latency_ms = baseline_mppi_.lastSolveMs();
    }

    {
      std::lock_guard<std::mutex> lock(outer_worker_mutex_);
      outer_worker_output_ = output;
    }
  }
}

std::shared_ptr<FSMState> AdhereController::getNextState(
  FSMStateName state_name) const
{
  for (const auto & [name, state] : fsm_states_) {
    if (name == state_name) {
      return state;
    }
  }
  // Unknown/INVALID falls back to the first registered state (IDLE_PASSIVE_NONE), matching the
  // previous switch's default case.
  return fsm_states_.front().second;
}

}  // namespace adhere_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  adhere_controller::AdhereController, controller_interface::ControllerInterface)
