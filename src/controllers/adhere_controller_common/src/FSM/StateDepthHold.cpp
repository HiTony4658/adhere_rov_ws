#include "adhere_controller_common/FSM/StateDepthHold.h"

#include <cmath>

#include "adhere_controller_common/common/mathTools.h"

namespace adhere_controller_common
{

namespace
{

// PID depth-hold gains (CLAUDE.md Phase 3.1 Stage 2.2/3.2 closed-loop tracking contract:
// <5mm steady-state error, minimal overshoot, converge within the 6s per-setpoint test
// window). Tuned against the plant's augmented mass (26 kg) and vertical Fossen damping
// (D_lin.z=25, D_quad.z=70 in fossen_hydrodynamics.hpp) with the max achievable allocated
// Fz (+-80N, see buildAllocationMatrix() below) comfortably inside actuator rails.
constexpr double kKpZ = 180.0;
constexpr double kKiZ = 95.0;
constexpr double kKdZ = 62.0;
constexpr double kIntegralMax = 180.0;  // N, anti-windup clamp on Ki*integral_z_

// Gentle attitude stabilization (angle-PD + rate damping) on roll/pitch/yaw. rov_wall.xml's
// tangential-thruster sites sit at body X=0.0, i.e. the SAME X coordinate as the CoM (rov_base's
// <inertial pos="0 0 0">) -- unlike the X=-0.10m lever-arm case CLAUDE.md Sec.5 describes for the
// full KAMR-MPPI wrench reconstruction, there is zero X-offset here, so tangential thrusters
// produce no parasitic pitch/yaw moment to compensate for; only the gentle restoring/damping
// terms below are needed.
constexpr double kKpAttitude = 6.0;
constexpr double kKdAttitude = 4.0;

constexpr double kThrusterMin = -40.0;
constexpr double kThrusterMax = 40.0;

// Exact 6x8 body-frame wrench allocation matrix B for rov_wall.xml's 8 thrusters (CLAUDE.md
// Phase 3.1 Stage 2.2 instruction to inspect the MJCF's thruster geometry/gear directions
// directly rather than reuse CLAUDE.md Sec.4.3's illustrative placeholder constants, which do
// not match this simplified model's actual corner/lever-arm geometry).
//
// Row order: [Fx, Fy, Fz, Tx, Ty, Tz]^T (body frame). Column order: thr_N1..thr_N4, thr_T5..
// thr_T8 (matches adhere_mujoco_hardware::kThrusterNames).
//
// Normal thrusters N1..N4: site at body (0, y_i, z_i), gear "-1 0 0 0 0 0" -> force -X_B * u_i.
// Cross product r x F with r=(0,y_i,z_i), F=(-u,0,0) gives (0, -z_i*u, y_i*u), i.e. Ty=-z_i*u,
// Tz=y_i*u.
//
// Tangential thrusters T5..T8: site at the same 4 corners, local +Z axis rotated to
// (0, -+cos30, +-sin30) per their MJCF <site zaxis="...">, gear "0 0 1 0 0 0" -> force
// u*(0, dy_i, dz_i). Cross product r x F with r=(0,y_i,z_i) gives
// (u*(y_i*dz_i - z_i*dy_i), 0, 0), i.e. Tx=u*(y_i*dz_i - z_i*dy_i), Ty=Tz=0 (r_x=0 kills the
// other two rows).
Mat68 buildAllocationMatrix()
{
  const double c30 = std::sqrt(3.0) / 2.0;  // cos(30 deg)
  const double s30 = 0.5;                   // sin(30 deg)

  // Corner order fixed to [TL, TR, BL, BR] = [N1/T5, N2/T6, N3/T7, N4/T8].
  const double y_corner[4] = {0.25, -0.25, 0.25, -0.25};
  const double z_corner[4] = {0.15, 0.15, -0.15, -0.15};
  // Tangential thrust direction (dy, dz) per corner, from rov_wall.xml's thr_T5..T8 zaxis.
  const double dy_corner[4] = {-c30, c30, -c30, c30};
  const double dz_corner[4] = {s30, s30, -s30, -s30};

  Mat68 b = Mat68::Zero();
  for (int i = 0; i < 4; ++i) {
    // Normal thruster column i.
    b(0, i) = -1.0;
    b(4, i) = -z_corner[i];
    b(5, i) = y_corner[i];

    // Tangential thruster column 4+i.
    const int col = 4 + i;
    b(1, col) = dy_corner[i];
    b(2, col) = dz_corner[i];
    b(3, col) = y_corner[i] * dz_corner[i] - z_corner[i] * dy_corner[i];
  }
  return b;
}

}  // namespace

StateDepthHold::StateDepthHold(CtrlInterfaces & ctrl_interfaces)
: FSMState(FSMStateName::DEPTH_HOLD, "state_depth_hold", ctrl_interfaces)
{
  // Eigen's CompleteOrthogonalDecomposition<Matrix<double,6,8>>::pseudoInverse() expression
  // template does not expose swapped (8,6) compile-time dimensions, so a direct fixed-size
  // Mat86 assignment trips EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE; routing through a dynamically-
  // sized intermediate (checked at runtime instead) sidesteps that without changing the result.
  const Eigen::MatrixXd pinv_dynamic =
    buildAllocationMatrix().completeOrthogonalDecomposition().pseudoInverse();
  b_pinv_ = pinv_dynamic;
}

void StateDepthHold::enter()
{
  // Only reached via a PASSIVE -> DEPTH_HOLD transition (checkChange() below never requests a
  // DEPTH_HOLD -> DEPTH_HOLD re-entry), so this is exactly the "entering from PASSIVE" case
  // CLAUDE.md Phase 3.1 Stage 2.2 says must reset the integrator; live target_z updates while
  // already in this state never call enter() again.
  integral_z_ = 0.0;
}

void StateDepthHold::run(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  const double pos_z = ctrl_interfaces_.odom_state_interface_[2].get().get_value();
  const Quat quat(
    ctrl_interfaces_.odom_state_interface_[3].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[4].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[5].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[6].get().get_value());
  const Vec3 vel_body(
    ctrl_interfaces_.odom_state_interface_[7].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[8].get().get_value(),
    ctrl_interfaces_.odom_state_interface_[9].get().get_value());
  const Vec3 omega_body(
    ctrl_interfaces_.imu_state_interface_[4].get().get_value(),
    ctrl_interfaces_.imu_state_interface_[5].get().get_value(),
    ctrl_interfaces_.imu_state_interface_[6].get().get_value());

  const RotMat r = quatToRotMat(quat);
  const Vec3 rpy = rotMatToRPY(r);
  const double vel_z_world = (r * vel_body)(2);

  const double target_z = ctrl_interfaces_.control_inputs_.target_z.load();
  const double e_z = target_z - pos_z;

  // Clamped anti-windup integral, derivative-on-measurement (CLAUDE.md Phase 3.1 Stage 2.2):
  // Kd acts on measured world vertical velocity, not d(e_z)/dt, so a live target_z retarget
  // (a setpoint step) never produces a derivative-kick transient.
  integral_z_ = saturation(integral_z_ + e_z * dt, -kIntegralMax / kKiZ, kIntegralMax / kKiZ);
  const double f_z_world_des = kKpZ * e_z + kKiZ * integral_z_ - kKdZ * vel_z_world;

  // Rotate the world-frame vertical force command into the body frame, then superimpose gentle
  // PD attitude stabilization torques (angle -> 0, rate damping).
  const Vec3 f_world_des(0.0, 0.0, f_z_world_des);
  const Vec3 f_body_des = r.transpose() * f_world_des;
  const Vec3 tau_body_des = -kKpAttitude * rpy - kKdAttitude * omega_body;

  Vec6 wrench_des;
  wrench_des << f_body_des, tau_body_des;

  const Vec8 u = b_pinv_ * wrench_des;
  for (int i = 0; i < 8; ++i) {
    ctrl_interfaces_.thruster_command_interface_[i].get().set_value(
      saturation(u(i), kThrusterMin, kThrusterMax));
  }
}

void StateDepthHold::exit()
{
}

FSMStateName StateDepthHold::checkChange()
{
  if (ctrl_interfaces_.control_inputs_.command.load() == 1) {
    return FSMStateName::PASSIVE;
  }
  return FSMStateName::DEPTH_HOLD;
}

}  // namespace adhere_controller_common
