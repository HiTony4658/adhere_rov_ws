#ifndef ADHERE_MUJOCO_HARDWARE__ADHERE_MUJOCO_HARDWARE_HPP_
#define ADHERE_MUJOCO_HARDWARE__ADHERE_MUJOCO_HARDWARE_HPP_

#include <array>
#include <memory>
#include <string>
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

  std::unique_ptr<FlowEnvironmentPlugin> flow_env_;
  std::shared_ptr<MujocoVisBridge> vis_bridge_;
  std::unique_ptr<MujocoViewer> viewer_;

  int body_id_ = -1;
  std::array<int, 4> pad_touch_sensor_adr_{};
  std::array<int, kNumThrusters> actuator_id_{};

  double sim_time_ = 0.0;
  long long step_count_ = 0;

  // Previous-tick world-frame linear velocity, used for the finite-difference linear
  // acceleration estimate reported on the imu_sensor state interfaces (no dedicated MJCF
  // accelerometer sensor exists in rov_wall.xml).
  Eigen::Vector3d prev_lin_vel_world_ = Eigen::Vector3d::Zero();
  bool have_prev_vel_ = false;
};

}  // namespace adhere_mujoco_hardware

#endif  // ADHERE_MUJOCO_HARDWARE__ADHERE_MUJOCO_HARDWARE_HPP_
