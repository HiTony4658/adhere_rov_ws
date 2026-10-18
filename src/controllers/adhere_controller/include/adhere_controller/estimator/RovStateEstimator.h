#ifndef ADHERE_CONTROLLER__ESTIMATOR__ROVSTATEESTIMATOR_H_
#define ADHERE_CONTROLLER__ESTIMATOR__ROVSTATEESTIMATOR_H_

#include <array>

#include "adhere_controller/common/mathTypes.h"

namespace adhere_controller
{

// RovStateEstimator (CLAUDE.md Phase 3.5 Stage 2.2): fuses the noisy multi-rate sensors
// (500Hz IMU, 50Hz Depth, 20Hz DVL) into a single pose/velocity/attitude estimate, running at
// 500Hz inside AdhereController::update() ahead of the inner-loop gate. Pure math, no ROS/
// MuJoCo dependency (same style as MultiRateDecimator.h/ThrusterAllocator.h) -- testable in
// isolation. Implementation lives in src/estimator/RovStateEstimator.cpp (CLAUDE.md Phase 3.7).
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
// compensated) at 20Hz and by Depth at 50Hz. Phase 3.7 adds a contact-aided pseudo-measurement
// (correctContact()) that arrests normal-axis drift once the hull is confirmed pressing on the
// wall by all 4 pad load cells.
class RovStateEstimator
{
public:
  // Re-latches the estimate to a known pose (e.g. ground truth at FSM-state enter()) and clears
  // all "have I seen this sample yet" edge-detection state. Called whenever estimation_mode
  // transitions into "sensor_fusion" so the filter does not have to converge from zero.
  void reset(const Vec3 & p0_world, const Vec3 & v0_body, const Quat & q0);

  // 500Hz prediction step. gyro_meas/accel_meas are the raw noisy IMU readings (bias/noise
  // already baked in by the sensor model -- the estimator does not need to know them
  // separately, exactly like a real AHRS/INS front end).
  void predict(const Vec3 & gyro_meas, const Vec3 & accel_meas, double dt);

  // 500Hz attitude correction: blends the gyro-integrated prediction with the noisy IMU/AHRS
  // orientation measurement (nlerp with hemisphere-alignment, cheap and accurate for the small
  // per-tick corrections this filter makes). Called every tick right after predict().
  void correctAttitude(const Quat & q_meas);

  // 50Hz Depth correction (CLAUDE.md Stage 2.2): call every tick with the latest noisy depth
  // reading -- internally no-ops unless the sample-and-hold value has actually changed (new
  // sample arrived), so the caller does not need to track the hardware's own decimation phase.
  void correctDepth(double z_meas, const Vec3 & r_depth_body);

  // 20Hz DVL correction with exact lever-arm compensation (CLAUDE.md Stage 2.2):
  // y_v_com = v_dvl_meas - omega_hat x r_dvl, then a steady-state Kalman update on v_hat_body_.
  // range_meas (acoustic wall distance, world X at the DVL site) additionally anchors position
  // drift along the wall-normal axis. Edge-detected exactly like correctDepth().
  void correctDvl(const Vec3 & v_dvl_meas_body, double range_meas, const Vec3 & r_dvl_body);

  // Phase 3.7 contact-aided wall-constraint pseudo-measurement update: call every 500Hz tick
  // with the 4 raw (unfiltered) loadcell_noisy readings. Internally EMA-filters them, and only
  // once ALL 4 filtered pad forces exceed `thresh` does it blend v_hat_body_.x() -> 0 and
  // p_hat_world_.x() -> x_wall_contact (eliminating low-frequency DVL integration drift during
  // prolonged wall adhesion, CLAUDE.md Phase 3.7 Stage 1.3).
  void correctContact(
    const std::array<double, 4> & pad_force_meas, double thresh, double x_wall_contact);

  const Vec3 & positionWorld() const { return p_hat_world_; }
  const Vec3 & velocityBody() const { return v_hat_body_; }
  const Vec3 & angularVelocityBody() const { return omega_hat_body_; }
  const Quat & orientation() const { return q_hat_; }
  Vec3 rpy() const;

private:
  // quatMul/normalizeQuat/smallAngleQuat/alignQuaternionHemisphere now live in mathTools.h
  // (Phase 3.6/3.7: hoisted out of here so StatePidHold's/SixDofPidController's quaternion math
  // reuses the exact same implementations instead of duplicating them).

  // Fixed correction gains (see class-level design note): analytically chosen from the
  // CLAUDE.md Stage 2.1 noise std values via the EMA steady-state-variance formula, verified
  // empirically in test_sensor_fusion_and_teleop.cpp against the <0.015 m/s / <15mm / <1.0deg
  // RMSE contract.
  static constexpr double kAttGain = 0.30;
  static constexpr double kDepthPosGain = 0.30;
  static constexpr double kDepthVelGain = 0.10;
  static constexpr double kDvlVelGain = 0.50;
  static constexpr double kDvlRangeGain = 0.30;
  // Phase 3.7 contact-aided update gains: EMA filter on the 4 raw loadcell channels, and the
  // pseudo-measurement blend gains on v_x/p_x once contact is confirmed.
  static constexpr double kContactForceFilterGain = 0.20;
  static constexpr double kContactVelGain = 0.50;
  static constexpr double kContactPosGain = 0.20;

  Vec3 p_hat_world_ = Vec3::Zero();
  Vec3 v_hat_body_ = Vec3::Zero();
  Vec3 omega_hat_body_ = Vec3::Zero();
  Quat q_hat_ = Quat(1.0, 0.0, 0.0, 0.0);

  bool have_last_depth_ = false;
  double last_depth_meas_ = 0.0;
  bool have_last_dvl_ = false;
  Vec3 last_dvl_meas_ = Vec3::Zero();

  bool have_pad_force_filtered_ = false;
  std::array<double, 4> pad_force_filtered_ = {0.0, 0.0, 0.0, 0.0};
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__ESTIMATOR__ROVSTATEESTIMATOR_H_
