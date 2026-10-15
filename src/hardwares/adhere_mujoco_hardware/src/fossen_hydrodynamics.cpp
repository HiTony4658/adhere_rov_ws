#include "adhere_mujoco_hardware/fossen_hydrodynamics.hpp"

#include <algorithm>
#include <cmath>

namespace adhere_mujoco_hardware
{

Wrench FossenHydrodynamics::compute(
  const RigidBodyState & state, double t, const HydroParams & params)
{
  const Eigen::Matrix3d R = state.orientation.toRotationMatrix();  // body -> world

  const Eigen::Vector3d & v_world = state.lin_vel_world;
  const Eigen::Vector3d & w_world = state.ang_vel_world;
  const Eigen::Vector3d w_body = R.transpose() * w_world;

  const Eigen::Vector3d current_world_t = params.currentWorld(t);
  const Eigen::Vector3d v_rel_world = v_world - current_world_t;
  const Eigen::Vector3d v_rel_body = R.transpose() * v_rel_world;

  // Coupled Morison drag, body frame: quadratic term scales by the total relative speed.
  const double V_rel_norm = v_rel_body.norm();
  const Eigen::Vector3d F_drag_body =
    -(params.D_lin.cwiseProduct(v_rel_body) + params.D_quad.cwiseProduct(v_rel_body) * V_rel_norm);
  const Eigen::Vector3d tau_drag_body =
    -(params.D_ang_lin.cwiseProduct(w_body) +
      params.D_ang_quad.cwiseProduct(w_body.cwiseProduct(w_body.cwiseAbs())));

  const Eigen::Vector3d F_drag_world = R * F_drag_body;
  const Eigen::Vector3d tau_drag_world = R * tau_drag_body;

  // Free-surface taper.
  const double com_z = state.com_pos_world.z();
  const double submerged_ratio = std::clamp(
    (params.z_surface + params.z_transition - com_z) / params.z_transition, 0.0, 1.0);

  // Buoyancy + added-mass gravity-bias correction (CLAUDE.md's 58.86 N figure at m_aug=26,
  // m=20, g=9.81).
  const double F_buoy_full = params.rho_water * params.displaced_volume * params.gravity;
  const double added_mass_gravity_bias =
    (params.augmented_mass - params.vehicle_mass) * params.gravity;
  const Eigen::Vector3d F_buoy_world(
    0.0, 0.0, (F_buoy_full + added_mass_gravity_bias) * submerged_ratio);

  // Restoring moment: r_(CoB<-CoM) x F_B.
  const Eigen::Vector3d r_cob_world = R * params.cob_offset_body;
  const Eigen::Vector3d tau_restore_world = r_cob_world.cross(F_buoy_world);

  // Venturi/Bernoulli wall-suction.
  const double x_com = state.com_pos_world.x();
  const double h_gap = std::max(x_com - params.wall_x_offset, 0.001);
  const double v_tangent_sq = v_rel_body.y() * v_rel_body.y() + v_rel_body.z() * v_rel_body.z();
  const double F_suction_x = -0.5 * params.rho_water * params.C_venturi * params.A_plate *
    v_tangent_sq * std::exp(-(h_gap - params.h_gap_nominal) / params.h0_decay);
  const Eigen::Vector3d F_suction_body(F_suction_x, 0.0, 0.0);
  const Eigen::Vector3d F_suction_world = R * F_suction_body;

  Wrench wrench;
  wrench.force_world =
    F_drag_world * submerged_ratio + F_buoy_world + F_suction_world * submerged_ratio;
  wrench.torque_world = tau_drag_world * submerged_ratio + tau_restore_world;
  return wrench;
}

}  // namespace adhere_mujoco_hardware
