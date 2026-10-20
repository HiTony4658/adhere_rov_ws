#include "adhere_controller/dynamics/ModelParams.h"

#include <string>
#include <vector>

namespace adhere_controller
{

ModelParams ModelParams::loadFromNode(const rclcpp::Node::SharedPtr & node)
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
  auto autoDeclareString = [&node](const std::string & name, const std::string & def) {
    if (!node->has_parameter(name)) {
      return node->declare_parameter<std::string>(name, def);
    }
    return node->get_parameter(name).get_value<std::string>();
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
  // Phase 3.8: 6-element Vec6 counterpart to declareVec3, for AdhereSmcGains' sliding-surface
  // vectors (order [x,y,z,roll,pitch,yaw], body frame).
  auto declareVec6 = [&autoDeclareVec](const std::string & name, Vec6 & dest) {
    std::vector<double> def(6);
    for (int i = 0; i < 6; ++i) {
      def[i] = dest(i);
    }
    const auto flat = autoDeclareVec(name, def);
    if (flat.size() == 6) {
      for (int i = 0; i < 6; ++i) {
        dest(i) = flat[i];
      }
    }
  };

  declareVec3Array("thruster_pos_xyz", p.thruster_pos, p.thruster_pos);
  declareVec3Array("thruster_dir_xyz", p.thruster_dir, p.thruster_dir);
  declareDoubleArray8("thruster_min", p.thruster_min, p.thruster_min);
  declareDoubleArray8("thruster_max", p.thruster_max, p.thruster_max);
  declareVec3Array4("pad_pos_xyz", p.pad_pos, p.pad_pos);
  p.kern_eps_margin = autoDeclareDouble("kern_eps_margin", p.kern_eps_margin);

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
  p.contact_force_thresh = autoDeclareDouble("contact_force_thresh", p.contact_force_thresh);

  declareVec3("dynamics.inertia_diag", p.dynamics.inertia_diag);
  declareVec3("dynamics.added_mass_lin", p.dynamics.added_mass_lin);
  declareVec3("dynamics.added_mass_ang", p.dynamics.added_mass_ang);

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
  p.pid_hold.max_tilt_deg = autoDeclareDouble("pid_hold.max_tilt_deg", p.pid_hold.max_tilt_deg);

  declareVec3("adhere_pid.kp_pos", p.adhere_pid.kp_pos);
  declareVec3("adhere_pid.ki_pos", p.adhere_pid.ki_pos);
  declareVec3("adhere_pid.kd_vel", p.adhere_pid.kd_vel);
  declareVec3("adhere_pid.kp_att", p.adhere_pid.kp_att);
  declareVec3("adhere_pid.ki_att", p.adhere_pid.ki_att);
  declareVec3("adhere_pid.kd_att", p.adhere_pid.kd_att);
  declareDoubleArray("adhere_pid.max_teleop_vel", p.adhere_pid.max_teleop_vel);
  p.adhere_pid.max_tilt_deg =
    autoDeclareDouble("adhere_pid.max_tilt_deg", p.adhere_pid.max_tilt_deg);
  p.adhere_pid.approach_speed =
    autoDeclareDouble("adhere_pid.approach_speed", p.adhere_pid.approach_speed);
  p.adhere_pid.dock_force_thresh =
    autoDeclareDouble("adhere_pid.dock_force_thresh", p.adhere_pid.dock_force_thresh);
  p.adhere_pid.x_wall = autoDeclareDouble("adhere_pid.x_wall", p.adhere_pid.x_wall);
  p.adhere_pid.x_dock_eps = autoDeclareDouble("adhere_pid.x_dock_eps", p.adhere_pid.x_dock_eps);
  p.adhere_pid.kp_force = autoDeclareDouble("adhere_pid.kp_force", p.adhere_pid.kp_force);
  p.adhere_pid.ki_force = autoDeclareDouble("adhere_pid.ki_force", p.adhere_pid.ki_force);
  p.adhere_pid.kd_x = autoDeclareDouble("adhere_pid.kd_x", p.adhere_pid.kd_x);
  p.adhere_pid.f_preload = autoDeclareDouble("adhere_pid.f_preload", p.adhere_pid.f_preload);
  p.adhere_pid.f_des = autoDeclareDouble("adhere_pid.f_des", p.adhere_pid.f_des);
  p.adhere_pid.integral_force_max =
    autoDeclareDouble("adhere_pid.integral_force_max", p.adhere_pid.integral_force_max);

  declareVec6("adhere_smc.lambda", p.adhere_smc.lambda);
  declareVec6("adhere_smc.kd", p.adhere_smc.kd);
  declareVec6("adhere_smc.k_switch", p.adhere_smc.k_switch);
  declareVec6("adhere_smc.phi", p.adhere_smc.phi);
  declareDoubleArray("adhere_smc.max_teleop_vel", p.adhere_smc.max_teleop_vel);
  p.adhere_smc.max_tilt_deg =
    autoDeclareDouble("adhere_smc.max_tilt_deg", p.adhere_smc.max_tilt_deg);
  p.adhere_smc.approach_speed =
    autoDeclareDouble("adhere_smc.approach_speed", p.adhere_smc.approach_speed);
  p.adhere_smc.dock_force_thresh =
    autoDeclareDouble("adhere_smc.dock_force_thresh", p.adhere_smc.dock_force_thresh);
  p.adhere_smc.x_wall = autoDeclareDouble("adhere_smc.x_wall", p.adhere_smc.x_wall);
  p.adhere_smc.x_dock_eps = autoDeclareDouble("adhere_smc.x_dock_eps", p.adhere_smc.x_dock_eps);
  p.adhere_smc.wall_preload_fx =
    autoDeclareDouble("adhere_smc.wall_preload_fx", p.adhere_smc.wall_preload_fx);

  p.adhere_stage.target_depth_m =
    autoDeclareDouble("adhere_target_depth_m", p.adhere_stage.target_depth_m);
  p.adhere_stage.target_y = autoDeclareDouble("adhere_target_y", p.adhere_stage.target_y);
  p.adhere_stage.standoff_x = autoDeclareDouble("adhere_standoff_x", p.adhere_stage.standoff_x);
  p.adhere_stage.depth_tol = autoDeclareDouble("adhere_depth_tol", p.adhere_stage.depth_tol);
  p.adhere_stage.vz_tol = autoDeclareDouble("adhere_vz_tol", p.adhere_stage.vz_tol);

  p.estimation_mode = autoDeclareString("estimation_mode", p.estimation_mode);
  declareVec3("sensor_mount.r_imu_xyz", p.sensor_mount.r_imu);
  declareVec3("sensor_mount.r_depth_xyz", p.sensor_mount.r_depth);
  declareVec3("sensor_mount.r_dvl_xyz", p.sensor_mount.r_dvl);

  return p;
}

}  // namespace adhere_controller
