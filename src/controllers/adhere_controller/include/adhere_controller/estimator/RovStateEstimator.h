#ifndef ADHERE_CONTROLLER__ESTIMATOR__ROVSTATEESTIMATOR_H_
#define ADHERE_CONTROLLER__ESTIMATOR__ROVSTATEESTIMATOR_H_

#include "adhere_controller/common/mathTools.h"
#include "adhere_controller/common/mathTypes.h"

namespace adhere_controller
{

// RovStateEstimator (CLAUDE.md Phase 3.5 Stage 2.2): fuses the noisy multi-rate sensors
// (500Hz IMU, 50Hz Depth, 20Hz DVL) into a single pose/velocity/attitude estimate, running at
// 500Hz inside AdhereController::update() ahead of the inner-loop gate. Pure math, no ROS/
// MuJoCo dependency (same header-only style as MultiRateDecimator.h/ThrusterAllocator.h) --
// testable in isolation.
//
// Design note: this is a DECOUPLED, fixed-gain (steady-state exponential-moving-average)
// Kalman/complementary filter per channel, not a single joint-covariance EKF. Each correction
// is algebraically a steady-state Kalman update y_hat += K*(y_meas - y_hat) with K chosen
// analytically from the CLAUDE.md Stage 2.1 noise std values via the EMA steady-state-variance
// formula Var(y_hat) = (K/(2-K)) * Var(meas) -- e.g. K=0.5 against DVL's 0.008 m/s std yields a
// steady-state velocity std of ~0.0046 m/s, comfortably inside the <0.015 m/s RMSE contract.
// Attitude is tracked directly in quaternion form (gyro-integrated prediction blended with the
// noisy IMU/AHRS quaternion measurement every tick, since that measurement arrives at the full
// 500Hz rate); position/velocity use IMU-accel dead-reckoning corrected by DVL (lever-arm
// compensated) at 20Hz and by Depth at 50Hz.
class RovStateEstimator
{
public:
  // Re-latches the estimate to a known pose (e.g. ground truth at FSM-state enter()) and clears
  // all "have I seen this sample yet" edge-detection state. Called whenever estimation_mode
  // transitions into "sensor_fusion" so the filter does not have to converge from zero.
  void reset(const Vec3 & p0_world, const Vec3 & v0_body, const Quat & q0)
  {
    p_hat_world_ = p0_world;
    v_hat_body_ = v0_body;
    q_hat_ = q0;
    omega_hat_body_.setZero();
    have_last_depth_ = false;
    have_last_dvl_ = false;
  }

  // 500Hz prediction step. gyro_meas/accel_meas are the raw noisy IMU readings (bias/noise
  // already baked in by the sensor model -- the estimator does not need to know them
  // separately, exactly like a real AHRS/INS front end).
  void predict(const Vec3 & gyro_meas, const Vec3 & accel_meas, double dt)
  {
    omega_hat_body_ = gyro_meas;

    // Quaternion kinematics: q_dot = 0.5 * q (x) [0, omega]. Small-angle exponential-map
    // increment, renormalized every tick (cheap and accurate at 500Hz/2ms steps).
    const Quat dq = smallAngleQuat(gyro_meas * dt);
    q_hat_ = normalizeQuat(quatMul(q_hat_, dq));

    // Body-frame velocity dead-reckoning from specific force/linear acceleration (already
    // gravity-free kinematic acceleration, matching AdhereMujocoHardware's IMU model).
    v_hat_body_ += accel_meas * dt;

    // World-frame position integration using the current attitude estimate.
    p_hat_world_ += quatToRotMat(q_hat_) * v_hat_body_ * dt;
  }

  // 500Hz attitude correction: blends the gyro-integrated prediction with the noisy IMU/AHRS
  // orientation measurement (nlerp with sign-alignment, cheap and accurate for the small
  // per-tick corrections this filter makes). Called every tick right after predict().
  void correctAttitude(const Quat & q_meas)
  {
    const double dot = q_hat_.dot(q_meas);
    const Quat q_meas_aligned = dot < 0.0 ? -q_meas : q_meas;
    q_hat_ = normalizeQuat((1.0 - kAttGain) * q_hat_ + kAttGain * q_meas_aligned);
  }

  // 50Hz Depth correction (CLAUDE.md Stage 2.2): call every tick with the latest noisy depth
  // reading -- internally no-ops unless the sample-and-hold value has actually changed (new
  // sample arrived), so the caller does not need to track the hardware's own decimation phase.
  void correctDepth(double z_meas, const Vec3 & r_depth_body)
  {
    if (have_last_depth_ && z_meas == last_depth_meas_) {
      return;
    }
    have_last_depth_ = true;
    last_depth_meas_ = z_meas;

    const RotMat r = quatToRotMat(q_hat_);
    const double z_pred = p_hat_world_.z() + (r * r_depth_body).z();
    const double residual = z_meas - z_pred;
    p_hat_world_.z() += kDepthPosGain * residual;

    // Secondary vertical-velocity correction: nudge the world-Z component of the current
    // velocity estimate toward the residual rate, then re-project into body frame.
    Vec3 v_world = r * v_hat_body_;
    v_world.z() += kDepthVelGain * residual;
    v_hat_body_ = r.transpose() * v_world;
  }

  // 20Hz DVL correction with exact lever-arm compensation (CLAUDE.md Stage 2.2):
  // y_v_com = v_dvl_meas - omega_hat x r_dvl, then a steady-state Kalman update on v_hat_body_.
  // range_meas (acoustic wall distance, world X at the DVL site) additionally anchors position
  // drift along the wall-normal axis. Edge-detected exactly like correctDepth().
  void correctDvl(const Vec3 & v_dvl_meas_body, double range_meas, const Vec3 & r_dvl_body)
  {
    if (have_last_dvl_ && v_dvl_meas_body == last_dvl_meas_) {
      return;
    }
    have_last_dvl_ = true;
    last_dvl_meas_ = v_dvl_meas_body;

    const Vec3 y_v_com = v_dvl_meas_body - omega_hat_body_.cross(r_dvl_body);
    v_hat_body_ += kDvlVelGain * (y_v_com - v_hat_body_);

    const RotMat r = quatToRotMat(q_hat_);
    const double x_pred = p_hat_world_.x() + (r * r_dvl_body).x();
    p_hat_world_.x() += kDvlRangeGain * (range_meas - x_pred);
  }

  const Vec3 & positionWorld() const { return p_hat_world_; }
  const Vec3 & velocityBody() const { return v_hat_body_; }
  const Vec3 & angularVelocityBody() const { return omega_hat_body_; }
  const Quat & orientation() const { return q_hat_; }
  Vec3 rpy() const { return rotMatToRPY(quatToRotMat(q_hat_)); }

private:
  static Quat quatMul(const Quat & a, const Quat & b)
  {
    Quat out;
    out(0) = a(0) * b(0) - a(1) * b(1) - a(2) * b(2) - a(3) * b(3);
    out(1) = a(0) * b(1) + a(1) * b(0) + a(2) * b(3) - a(3) * b(2);
    out(2) = a(0) * b(2) - a(1) * b(3) + a(2) * b(0) + a(3) * b(1);
    out(3) = a(0) * b(3) + a(1) * b(2) - a(2) * b(1) + a(3) * b(0);
    return out;
  }

  static Quat normalizeQuat(const Quat & q)
  {
    const double n = q.norm();
    return n > 1e-12 ? (q / n).eval() : Quat(1.0, 0.0, 0.0, 0.0);
  }

  static Quat smallAngleQuat(const Vec3 & theta)
  {
    Quat dq;
    dq << 1.0, 0.5 * theta.x(), 0.5 * theta.y(), 0.5 * theta.z();
    return normalizeQuat(dq);
  }

  // Fixed correction gains (see class-level design note): analytically chosen from the
  // CLAUDE.md Stage 2.1 noise std values via the EMA steady-state-variance formula, verified
  // empirically in test_sensor_fusion_and_teleop.cpp against the <0.015 m/s / <15mm / <1.0deg
  // RMSE contract.
  static constexpr double kAttGain = 0.30;
  static constexpr double kDepthPosGain = 0.30;
  static constexpr double kDepthVelGain = 0.10;
  static constexpr double kDvlVelGain = 0.50;
  static constexpr double kDvlRangeGain = 0.30;

  Vec3 p_hat_world_ = Vec3::Zero();
  Vec3 v_hat_body_ = Vec3::Zero();
  Vec3 omega_hat_body_ = Vec3::Zero();
  Quat q_hat_ = Quat(1.0, 0.0, 0.0, 0.0);

  bool have_last_depth_ = false;
  double last_depth_meas_ = 0.0;
  bool have_last_dvl_ = false;
  Vec3 last_dvl_meas_ = Vec3::Zero();
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__ESTIMATOR__ROVSTATEESTIMATOR_H_
