#include "adhere_controller/estimator/RovStateEstimator.h"

#include "adhere_controller/common/mathTools.h"

namespace adhere_controller
{

void RovStateEstimator::reset(const Vec3 & p0_world, const Vec3 & v0_body, const Quat & q0)
{
  p_hat_world_ = p0_world;
  v_hat_body_ = v0_body;
  q_hat_ = q0;
  omega_hat_body_.setZero();
  have_last_depth_ = false;
  have_last_dvl_ = false;
  have_pad_force_filtered_ = false;
}

void RovStateEstimator::predict(const Vec3 & gyro_meas, const Vec3 & accel_meas, double dt)
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

void RovStateEstimator::correctAttitude(const Quat & q_meas)
{
  const Quat q_meas_aligned = alignQuaternionHemisphere(q_hat_, q_meas);
  q_hat_ = normalizeQuat((1.0 - gains_.att_gain) * q_hat_ + gains_.att_gain * q_meas_aligned);
}

void RovStateEstimator::correctDepth(double z_meas, const Vec3 & r_depth_body)
{
  if (have_last_depth_ && z_meas == last_depth_meas_) {
    return;
  }
  have_last_depth_ = true;
  last_depth_meas_ = z_meas;

  const RotMat r = quatToRotMat(q_hat_);
  const double z_pred = p_hat_world_.z() + (r * r_depth_body).z();
  const double residual = z_meas - z_pred;
  p_hat_world_.z() += gains_.depth_pos_gain * residual;

  // Secondary vertical-velocity correction: nudge the world-Z component of the current
  // velocity estimate toward the residual rate, then re-project into body frame.
  Vec3 v_world = r * v_hat_body_;
  v_world.z() += gains_.depth_vel_gain * residual;
  v_hat_body_ = r.transpose() * v_world;
}

void RovStateEstimator::correctDvl(
  const Vec3 & v_dvl_meas_body, double range_meas, const Vec3 & r_dvl_body)
{
  if (have_last_dvl_ && v_dvl_meas_body == last_dvl_meas_) {
    return;
  }
  have_last_dvl_ = true;
  last_dvl_meas_ = v_dvl_meas_body;

  const Vec3 y_v_com = v_dvl_meas_body - omega_hat_body_.cross(r_dvl_body);
  v_hat_body_ += gains_.dvl_vel_gain * (y_v_com - v_hat_body_);

  const RotMat r = quatToRotMat(q_hat_);
  const double x_pred = p_hat_world_.x() + (r * r_dvl_body).x();
  p_hat_world_.x() += gains_.dvl_range_gain * (range_meas - x_pred);
}

void RovStateEstimator::correctContact(
  const std::array<double, 4> & pad_force_meas, double thresh, double x_wall_contact)
{
  if (!have_pad_force_filtered_) {
    pad_force_filtered_ = pad_force_meas;
    have_pad_force_filtered_ = true;
  } else {
    for (int i = 0; i < 4; ++i) {
      pad_force_filtered_[i] = (1.0 - gains_.contact_force_filter_gain) * pad_force_filtered_[i] +
        gains_.contact_force_filter_gain * pad_force_meas[i];
    }
  }

  const bool all_in_contact =
    pad_force_filtered_[0] > thresh && pad_force_filtered_[1] > thresh &&
    pad_force_filtered_[2] > thresh && pad_force_filtered_[3] > thresh;
  if (!all_in_contact) {
    return;
  }

  v_hat_body_.x() = (1.0 - gains_.contact_vel_gain) * v_hat_body_.x();
  p_hat_world_.x() += gains_.contact_pos_gain * (x_wall_contact - p_hat_world_.x());
}

}  // namespace adhere_controller
