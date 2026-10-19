#ifndef ADHERE_CONTROLLER__ALGORITHMS__BASELINE_MPPI__BASELINEMPPICONFIG_H_
#define ADHERE_CONTROLLER__ALGORITHMS__BASELINE_MPPI__BASELINEMPPICONFIG_H_

#include <array>
#include <cstdint>

#include "rclcpp/node.hpp"

namespace adhere_controller
{
namespace baseline_mppi
{

// Host-side Baseline MPPI hyperparameters (CLAUDE.md Phase 4.1 Sec.1/2.1.1): deliberately NOT
// part of dynamics::ModelParams -- ModelParams stays scoped to physical-truth parameters shared
// by every algorithm, while each planning algorithm (baseline_mppi today, kamr_mppi later) owns
// its own config schema and loader. Parsed from robot_control.yaml's
// `adhere_controller.ros__parameters.baseline_mppi.*` block by loadFromNode() below, which
// mirrors ModelParams::loadFromNode()'s auto_declare-with-has_parameter-guard pattern
// (dynamics/ModelParams.cpp) but is intentionally a separate, independent parser.
struct BaselineMppiConfig
{
  int num_rollouts = 1024;    // K
  int horizon = 30;           // H
  double dt = 0.04;           // s, matches the 25Hz outer-loop period (outer_decimation_=20)
  double lambda = 10.0;       // MPPI inverse temperature
  std::array<double, 6> sigma_tau = {40.0, 30.0, 30.0, 8.0, 8.0, 8.0};  // [Fx,Fy,Fz,Tx,Ty,Tz] N/N*m
  std::array<double, 3> q_pos = {800.0, 800.0, 800.0};
  double q_quat = 40.0;
  std::array<double, 6> q_vel = {8.0, 8.0, 8.0, 1.0, 1.0, 1.0};
  double r_u = 1.0e-3;
  std::array<double, 6> r_dtau = {5.0e-3, 5.0e-3, 5.0e-3, 5.0e-3, 5.0e-3, 5.0e-3};
  double terminal_mult = 5.0;
  double x_wall = 0.0;              // m, world X of the wall plane (CLAUDE.md Sec.4.1)
  double wall_k = 4000.0;           // N/m, rollout-only unilateral contact penalty stiffness
  double wall_b = 200.0;            // N*s/m, rollout-only contact penalty damping
  double preload_fx = 30.0;         // N, constant pressing bias once a rollout reaches the wall
  uint64_t seed = 88172645463325252ULL;  // curand Philox seed (fixed -> deterministic solve())

  static BaselineMppiConfig loadFromNode(const rclcpp::Node::SharedPtr & node);
};

}  // namespace baseline_mppi
}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__ALGORITHMS__BASELINE_MPPI__BASELINEMPPICONFIG_H_
