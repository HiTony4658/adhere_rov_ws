#ifndef ADHERE_MUJOCO_HARDWARE__ADHERE_MUJOCO_HARDWARE_HPP_
#define ADHERE_MUJOCO_HARDWARE__ADHERE_MUJOCO_HARDWARE_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "adhere_mujoco_hardware/flow_environment_plugin.hpp"
#include "adhere_mujoco_hardware/fossen_hydrodynamics.hpp"
#include "adhere_mujoco_hardware/mujoco_vis_bridge.hpp"
#include "adhere_mujoco_hardware/mujoco_viewer.hpp"

namespace adhere_mujoco_hardware
{

// Index layout for hw_commands_ / the thruster-readback half of hw_states_: 4 normal
// ("pressing") thrusters followed by 4 tangential thrusters, matching mjcf/rov_wall.xml's
// actuator order and CLAUDE.md Sec.4.3.
constexpr int kNumThrusters = 8;
constexpr std::array<const char *, kNumThrusters> kThrusterNames = {
  "thr_N1", "thr_N2", "thr_N3", "thr_N4", "thr_T5", "thr_T6", "thr_T7", "thr_T8"};
constexpr std::array<const char *, 4> kPadNames = {"pad_TL", "pad_TR", "pad_BL", "pad_BR"};

// 36 total state interfaces: 8 thruster readback + 10 IMU + 10 odom + 8 pad-contact.
constexpr int kNumStates = 36;

// AdhereMujocoHardware: a 500Hz hardware_interface::SystemInterface (ROS 2 Foxy) that owns
// the master MuJoCo mjModel/mjData, the FossenHydrodynamics port, the active
// FlowEnvironmentPlugin, and the raw hw_states_[36]/hw_commands_[8] buffers. Foxy's
// SystemInterface has no time/period arguments on read()/write() -- dt is taken from
// mjModel::opt.timestep (CLAUDE.md Sec.2).
class AdhereMujocoHardware
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  ~AdhereMujocoHardware() override;

  hardware_interface::return_type configure(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type start() override;
  hardware_interface::return_type stop() override;

  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

  // Const accessors used by adhere_mujoco_node for its 100Hz TF/Odom/Marker publishing and
  // by MujocoViewer's construction. Safe to call only after configure()/start().
  const mjModel * model() const { return model_; }
  std::shared_ptr<MujocoVisBridge> visBridge() const { return vis_bridge_; }

  // Runtime wave-maker switch (CLAUDE.md Phase 2.5 Stage 2.3/3.2): thread-safe, callable from
  // any thread (e.g. adhere_mujoco_node's /rov/wave_mode_cmd subscriber callback, which runs
  // on the executor thread, not the 500Hz physics thread). mode must be 1 (CalmWater) or 2
  // (StationKeepingGust); anything else is ignored. Selecting mode 2 always re-latches
  // t_trigger to the current sim time, so repeated "2" presses restart the gust clock.
  void setWaveMode(int mode);
  int waveMode() const { return wave_mode_.load(); }

  // Single-instance back-pointer accessor (see the mjcb_passive rationale in
  // adhere_mujoco_hardware.cpp): lets adhere_mujoco_node's wave-mode subscriber reach the one
  // live AdhereMujocoHardware instance without threading a pointer through pluginlib/
  // ResourceManager. Also used by adhere_mujoco_node to call setHeadless() between
  // ResourceManager construction (which runs configure()) and start_components() (which runs
  // start()), since pure-xacro decoupling (CLAUDE.md Phase 2.6 Stage 2.1) means headless is no
  // longer a <ros2_control> hardware_parameters entry.
  static AdhereMujocoHardware * activeInstance();

  // Must be called after configure() (i.e. after the ResourceManager that owns this instance
  // has been constructed) and before start_components() triggers start(), since start() is
  // what constructs MujocoViewer with the requested headless flag.
  void setHeadless(bool headless) { headless_ = headless; }

private:
  // Installed as the body of the global mjcb_passive hook (see adhere_mujoco_hardware.cpp);
  // computes and injects the Fossen hydrodynamic wrench at exactly the point in mj_step
  // where the Python reference's mjcb_passive callback did, for numerical parity.
  void applyHydrodynamics(const mjModel * m, mjData * d);

  // State slot offsets within hw_states_[36].
  enum StateOffset
  {
    kThrusterReadback = 0,       // [0, 8)
    kImu = kNumThrusters,        // [8, 18)
    kOdom = kImu + 10,           // [18, 28)
    kPadContact = kOdom + 10,    // [28, 36)
  };

  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;

  std::array<double, kNumStates> hw_states_{};
  std::array<double, kNumThrusters> hw_commands_{};
  // First-order ESC lag filter state (Phase 2.11 Section 4), written every write() tick
  // regardless of the lag checkbox so disabling it mid-flight is an instant passthrough.
  std::array<double, kNumThrusters> applied_thrust_{};

  // Both wave-environment plugins are always constructed; the runtime switch below (instead
  // of a single `flow_env_` selected once at configure()) is what makes "1"/"2" console input
  // take effect without restarting the node.
  std::unique_ptr<FlowEnvironmentPlugin> calm_plugin_;
  std::unique_ptr<FlowEnvironmentPlugin> gust_plugin_;
  std::atomic<int> wave_mode_{1};              // 1 = CalmWater (default), 2 = StationKeepingGust
  std::atomic<double> wave_trigger_time_{0.0};  // sim time t_trigger latched when mode 2 is selected
  std::atomic<double> sim_time_atomic_{0.0};    // lock-free mirror of sim_time_ for setWaveMode()

  std::shared_ptr<MujocoVisBridge> vis_bridge_;
  std::unique_ptr<MujocoViewer> viewer_;

  int body_id_ = -1;
  std::array<int, 4> pad_touch_sensor_adr_{};
  std::array<int, kNumThrusters> actuator_id_{};

  // Body-local (y, z) of the 4 pad sites, resolved at configure() time from
  // model_->site_pos[pad_*_site] instead of being hardcoded (CLAUDE.md Phase 2.6 Stage 2.1
  // bug #6): used by read()'s v_sep,i = v_x,b + omega_y*z_i - omega_z*y_i formula.
  std::array<std::pair<double, double>, 4> pad_yz_{};

  // Geometry-driven hydrodynamics engine (CLAUDE.md Phase 2.10 Section B): built once in
  // configure() from model_'s own geoms via HydroGeometryModel::buildFromMjModel(), cached
  // for the lifetime of this instance. hydro_mode_ mirrors vis_bridge_->hydro_mode, read once
  // per write() tick -- no edge-detection needed, it's a plain mode selector.
  HydroGeometryModel hydro_geometry_model_;
  std::atomic<HydroModelMode> hydro_mode_{HydroModelMode::kParametricLumped};

  bool headless_ = false;

  double sim_time_ = 0.0;
  long long step_count_ = 0;        // real physics-step counter (frozen while paused)
  long long write_tick_count_ = 0;  // increments every write() call, pause or not (Bug 2 fix)

  // Previous-tick world-frame linear velocity, used for the finite-difference linear
  // acceleration estimate reported on the imu_sensor state interfaces (no dedicated MJCF
  // accelerometer sensor exists in rov_wall.xml).
  Eigen::Vector3d prev_lin_vel_world_ = Eigen::Vector3d::Zero();
  bool have_prev_vel_ = false;

  // Paused-IMU latch (Phase 2.11 Section 3): read() is called every 500Hz tick regardless of
  // pause state, but the finite-difference accel must only be recomputed/advanced when
  // step_count_ actually changes -- otherwise a paused sim immediately zeroes IMU accel on the
  // very next read() (lin_vel_world - prev_lin_vel_world_ == 0 while step_count_ is frozen).
  long long last_accel_step_count_ = -1;
  Eigen::Vector3d last_accel_body_ = Eigen::Vector3d::Zero();

  // True 500Hz cumulative contact/Rhombic-Kern telemetry counters (Phase 2.11 Section 2),
  // accumulated on every real physics step (not just the ~60Hz vis-bridge snapshot cadence) so
  // transient <5ms contact events are never aliased away. Reset alongside did_reset or an
  // explicit vis_bridge_->reset_stats_request.
  long long total_eval_steps_ = 0;
  long long liftoff_steps_ = 0;
  long long liftoff_events_ = 0;
  long long corner_trap_steps_ = 0;
  bool prev_is_liftoff_ = false;
};

}  // namespace adhere_mujoco_hardware

#endif  // ADHERE_MUJOCO_HARDWARE__ADHERE_MUJOCO_HARDWARE_HPP_
