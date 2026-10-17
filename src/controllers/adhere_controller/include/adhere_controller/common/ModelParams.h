#ifndef ADHERE_CONTROLLER__COMMON__MODELPARAMS_H_
#define ADHERE_CONTROLLER__COMMON__MODELPARAMS_H_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/node.hpp"

#include "adhere_controller/common/mathTypes.h"

namespace adhere_controller
{

// Runtime-loaded, model-decoupled physical/control parameters (CLAUDE.md Phase 3.3 Sec.3):
// everything here used to be a `constexpr`/`const Vec3` baked into StateDepthHold.cpp/
// StateOpenLoop.cpp/StatePidHold.cpp/ThrusterAllocator.h. Switching to a robot with different
// thruster geometry, mass, or gains now only requires editing
// `<pkg_description>/config/robot_control.yaml` -- zero adhere_controller C++ changes or
// recompilation.
struct ModelParams
{
  struct DepthHoldGains
  {
    double kp_z = 180.0;
    double ki_z = 95.0;
    double kd_z = 62.0;
    double max_heave_vel = 0.30;  // m/s, Xbox left-stick full-deflection target_z_ rate
    double kp_att = 6.0;
    double kd_att = 4.0;
  };

  struct OpenLoopGains
  {
    // [Fx, Fy, Fz, Tx, Ty, Tz] max wrench magnitude at full stick deflection (CLAUDE.md
    // Phase 3.2 Stage 2.2.3). Only indices 0 (Fx), 1 (Fy), 2 (Fz), 5 (Tz) are consumed today.
    std::array<double, 6> max_wrench = {60.0, 50.0, 50.0, 5.0, 5.0, 8.0};
  };

  struct PidHoldGains
  {
    Vec3 kp_pos = Vec3(120.0, 120.0, 120.0);
    Vec3 ki_pos = Vec3(10.0, 10.0, 15.0);
    Vec3 kd_vel = Vec3(60.0, 60.0, 60.0);
    Vec3 kp_att = Vec3(40.0, 40.0, 25.0);
    Vec3 ki_att = Vec3(2.0, 2.0, 2.0);
    Vec3 kd_att = Vec3(15.0, 15.0, 12.0);
    // [vx_max, vy_max, vz_max, omega_z_max] stick-driven setpoint-move rates.
    std::array<double, 4> max_teleop_vel = {0.25, 0.25, 0.25, 0.6};
  };

  // Thruster geometry/limits (body frame, relative to CoM) for dynamically building
  // B in R^{6x8} / B^+ in R^{8x6} (ThrusterAllocator.h). Default values are rov_wall.xml's
  // REAL geometry (thr_N1..N4 normal thrusters at the 4 corners, gear "-1 0 0 0 0 0"; thr_T5..
  // T8 tangential thrusters at the same 4 corners, direction from each site's zaxis) -- see
  // CLAUDE.md Phase 3.2's documented deviation from Sec.4.3's illustrative lever-arm placeholder.
  std::array<Vec3, 8> thruster_pos = {
    Vec3(0.0, 0.25, 0.15), Vec3(0.0, -0.25, 0.15), Vec3(0.0, 0.25, -0.15),
    Vec3(0.0, -0.25, -0.15), Vec3(0.0, 0.25, 0.15), Vec3(0.0, -0.25, 0.15),
    Vec3(0.0, 0.25, -0.15), Vec3(0.0, -0.25, -0.15)};
  std::array<Vec3, 8> thruster_dir = {
    Vec3(-1.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0),
    Vec3(0.0, -0.8660254037844386, 0.5), Vec3(0.0, 0.8660254037844386, 0.5),
    Vec3(0.0, -0.8660254037844386, -0.5), Vec3(0.0, 0.8660254037844386, -0.5)};
  std::array<double, 8> thruster_min = {-40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0, -40.0};
  std::array<double, 8> thruster_max = {40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0};

  // Contact pad geometry (body frame), for Rhombic Kern / future WBC. Unused by the current
  // FSM states.
  std::array<Vec3, 4> pad_pos = {
    Vec3(-0.10, 0.25, 0.15), Vec3(-0.10, -0.25, 0.15), Vec3(-0.10, 0.25, -0.15),
    Vec3(-0.10, -0.25, -0.15)};

  // Hydrostatic & Fossen hydrodynamic feedforward parameters.
  double rho_water = 1000.0;
  double gravity = 9.81;
  double vehicle_mass = 20.0;
  double augmented_mass = 26.0;
  double displaced_volume = 0.0202;
  Vec3 cob_offset_body = Vec3(0.0, 0.0, 0.05);
  double z_surface = 0.0;
  double z_transition = 0.4;
  Vec3 D_lin = Vec3(30.0, 20.0, 25.0);
  Vec3 D_quad = Vec3(140.0, 50.0, 70.0);
  Vec3 D_ang_lin = Vec3(8.0, 8.0, 10.0);
  Vec3 D_ang_quad = Vec3(15.0, 15.0, 20.0);

  // Workspace safety bounds (world frame).
  double workspace_z_min = -9.80;
  double workspace_z_max = -0.15;
  double workspace_x_min = 0.115;

  DepthHoldGains depth_hold;
  OpenLoopGains open_loop;
  PidHoldGains pid_hold;

  // Declares/reads every field above as a ROS parameter on `node` (mirrors the existing
  // auto_declare<int>("cm_update_rate", ...) pattern already used for the multi-rate
  // parameters in AdhereController::on_configure()). Flat vector/scalar parameters are
  // declared with this struct's compiled-in defaults as their ROS defaults, so a
  // robot_control.yaml that omits a key still yields a fully-specified ModelParams.
  static ModelParams loadFromNode(const rclcpp::Node::SharedPtr & node)
  {
    ModelParams p;

    // Mirrors controller_interface::ControllerInterfaceBase::auto_declare(): the controller's
    // node is constructed by controller_manager with automatically_declare_parameters_from_
    // overrides enabled, so every key present in robot_control.yaml is ALREADY declared by
    // the time on_configure() runs -- calling declare_parameter() again on those names throws
    // rclcpp::exceptions::ParameterAlreadyDeclaredException. has_parameter() guards against that.
    auto autoDeclareVec = [&node](const std::string & name, const std::vector<double> & def) {
      if (!node->has_parameter(name)) {
        return node->declare_parameter<std::vector<double>>(name, def);
      }
      return node->get_parameter(name).get_value<std::vector<double>>();
    };
    auto autoDeclareDouble = [&node](const std::string & name, double def) {
      if (!node->has_parameter(name)) {
        return node->declare_parameter<double>(name, def);
      }
      return node->get_parameter(name).get_value<double>();
    };

    auto declareVec3Array = [&autoDeclareVec](
      const std::string & name, std::array<Vec3, 8> & dest, const std::array<Vec3, 8> & def) {
      std::vector<double> flat_def;
      flat_def.reserve(def.size() * 3);
      for (const auto & v : def) {
        flat_def.push_back(v.x());
        flat_def.push_back(v.y());
        flat_def.push_back(v.z());
      }
      const auto flat = autoDeclareVec(name, flat_def);
      for (size_t i = 0; i < dest.size() && 3 * i + 2 < flat.size(); ++i) {
        dest[i] = Vec3(flat[3 * i], flat[3 * i + 1], flat[3 * i + 2]);
      }
    };
    auto declareVec3Array4 = [&autoDeclareVec](
      const std::string & name, std::array<Vec3, 4> & dest, const std::array<Vec3, 4> & def) {
      std::vector<double> flat_def;
      flat_def.reserve(def.size() * 3);
      for (const auto & v : def) {
        flat_def.push_back(v.x());
        flat_def.push_back(v.y());
        flat_def.push_back(v.z());
      }
      const auto flat = autoDeclareVec(name, flat_def);
      for (size_t i = 0; i < dest.size() && 3 * i + 2 < flat.size(); ++i) {
        dest[i] = Vec3(flat[3 * i], flat[3 * i + 1], flat[3 * i + 2]);
      }
    };
    auto declareDoubleArray8 = [&autoDeclareVec](
      const std::string & name, std::array<double, 8> & dest, const std::array<double, 8> & def) {
      const std::vector<double> flat_def(def.begin(), def.end());
      const auto flat = autoDeclareVec(name, flat_def);
      for (size_t i = 0; i < dest.size() && i < flat.size(); ++i) {
        dest[i] = flat[i];
      }
    };
    auto declareVec3 = [&autoDeclareVec](const std::string & name, Vec3 & dest) {
      const auto flat = autoDeclareVec(name, {dest.x(), dest.y(), dest.z()});
      if (flat.size() == 3) {
        dest = Vec3(flat[0], flat[1], flat[2]);
      }
    };
    auto declareDoubleArray = [&autoDeclareVec](
      const std::string & name, auto & dest) {
      const std::vector<double> def(dest.begin(), dest.end());
      const auto flat = autoDeclareVec(name, def);
      for (size_t i = 0; i < dest.size() && i < flat.size(); ++i) {
        dest[i] = flat[i];
      }
    };

    declareVec3Array("thruster_pos_xyz", p.thruster_pos, p.thruster_pos);
    declareVec3Array("thruster_dir_xyz", p.thruster_dir, p.thruster_dir);
    declareDoubleArray8("thruster_min", p.thruster_min, p.thruster_min);
    declareDoubleArray8("thruster_max", p.thruster_max, p.thruster_max);
    declareVec3Array4("pad_pos_xyz", p.pad_pos, p.pad_pos);

    p.rho_water = autoDeclareDouble("rho_water", p.rho_water);
    p.gravity = autoDeclareDouble("gravity", p.gravity);
    p.vehicle_mass = autoDeclareDouble("vehicle_mass", p.vehicle_mass);
    p.augmented_mass = autoDeclareDouble("augmented_mass", p.augmented_mass);
    p.displaced_volume = autoDeclareDouble("displaced_volume", p.displaced_volume);
    declareVec3("cob_offset_body", p.cob_offset_body);
    p.z_surface = autoDeclareDouble("z_surface", p.z_surface);
    p.z_transition = autoDeclareDouble("z_transition", p.z_transition);
    declareVec3("D_lin", p.D_lin);
    declareVec3("D_quad", p.D_quad);
    declareVec3("D_ang_lin", p.D_ang_lin);
    declareVec3("D_ang_quad", p.D_ang_quad);

    p.workspace_z_min = autoDeclareDouble("workspace_z_min", p.workspace_z_min);
    p.workspace_z_max = autoDeclareDouble("workspace_z_max", p.workspace_z_max);
    p.workspace_x_min = autoDeclareDouble("workspace_x_min", p.workspace_x_min);

    p.depth_hold.kp_z = autoDeclareDouble("depth_hold.kp_z", p.depth_hold.kp_z);
    p.depth_hold.ki_z = autoDeclareDouble("depth_hold.ki_z", p.depth_hold.ki_z);
    p.depth_hold.kd_z = autoDeclareDouble("depth_hold.kd_z", p.depth_hold.kd_z);
    p.depth_hold.max_heave_vel =
      autoDeclareDouble("depth_hold.max_heave_vel", p.depth_hold.max_heave_vel);
    p.depth_hold.kp_att = autoDeclareDouble("depth_hold.kp_att", p.depth_hold.kp_att);
    p.depth_hold.kd_att = autoDeclareDouble("depth_hold.kd_att", p.depth_hold.kd_att);

    declareDoubleArray("open_loop.max_wrench", p.open_loop.max_wrench);

    declareVec3("pid_hold.kp_pos", p.pid_hold.kp_pos);
    declareVec3("pid_hold.ki_pos", p.pid_hold.ki_pos);
    declareVec3("pid_hold.kd_vel", p.pid_hold.kd_vel);
    declareVec3("pid_hold.kp_att", p.pid_hold.kp_att);
    declareVec3("pid_hold.ki_att", p.pid_hold.ki_att);
    declareVec3("pid_hold.kd_att", p.pid_hold.kd_att);
    declareDoubleArray("pid_hold.max_teleop_vel", p.pid_hold.max_teleop_vel);

    return p;
  }
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__MODELPARAMS_H_
