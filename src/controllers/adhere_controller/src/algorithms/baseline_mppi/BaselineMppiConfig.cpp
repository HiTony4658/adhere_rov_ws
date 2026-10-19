#include "adhere_controller/algorithms/baseline_mppi/BaselineMppiConfig.h"

#include <string>
#include <vector>

namespace adhere_controller
{
namespace baseline_mppi
{

namespace
{

// Mirrors dynamics/ModelParams.cpp's autoDeclare* lambdas (CLAUDE.md Phase 3.3): the
// controller's node auto-declares every robot_control.yaml key before on_configure() runs, so
// has_parameter() must guard against re-declaring (which would throw
// ParameterAlreadyDeclaredException).
double autoDeclareDouble(const rclcpp::Node::SharedPtr & node, const std::string & name, double def)
{
  if (!node->has_parameter(name)) {
    return node->declare_parameter<double>(name, def);
  }
  return node->get_parameter(name).get_value<double>();
}

int autoDeclareInt(const rclcpp::Node::SharedPtr & node, const std::string & name, int def)
{
  if (!node->has_parameter(name)) {
    return static_cast<int>(node->declare_parameter<int>(name, def));
  }
  return static_cast<int>(node->get_parameter(name).get_value<int>());
}

// Dedicated int64 path for the curand seed (CLAUDE.md Phase 4.1): rclcpp's integer parameter
// type is backed by int64_t, and 88172645463325252's default value overflows a 32-bit int, so
// this must not round-trip through autoDeclareInt() above.
int64_t autoDeclareInt64(
  const rclcpp::Node::SharedPtr & node, const std::string & name, int64_t def)
{
  if (!node->has_parameter(name)) {
    return node->declare_parameter<int64_t>(name, def);
  }
  return node->get_parameter(name).get_value<int64_t>();
}

template <size_t N>
void autoDeclareArray(
  const rclcpp::Node::SharedPtr & node, const std::string & name, std::array<double, N> & dest)
{
  std::vector<double> flat_def(dest.begin(), dest.end());
  std::vector<double> flat;
  if (!node->has_parameter(name)) {
    flat = node->declare_parameter<std::vector<double>>(name, flat_def);
  } else {
    flat = node->get_parameter(name).get_value<std::vector<double>>();
  }
  for (size_t i = 0; i < N && i < flat.size(); ++i) {
    dest[i] = flat[i];
  }
}

}  // namespace

BaselineMppiConfig BaselineMppiConfig::loadFromNode(const rclcpp::Node::SharedPtr & node)
{
  BaselineMppiConfig c;

  c.num_rollouts = autoDeclareInt(node, "baseline_mppi.num_rollouts", c.num_rollouts);
  c.horizon = autoDeclareInt(node, "baseline_mppi.horizon", c.horizon);
  c.dt = autoDeclareDouble(node, "baseline_mppi.dt", c.dt);
  c.lambda = autoDeclareDouble(node, "baseline_mppi.lambda", c.lambda);
  autoDeclareArray(node, "baseline_mppi.sigma_tau", c.sigma_tau);
  autoDeclareArray(node, "baseline_mppi.q_pos", c.q_pos);
  c.q_quat = autoDeclareDouble(node, "baseline_mppi.q_quat", c.q_quat);
  autoDeclareArray(node, "baseline_mppi.q_vel", c.q_vel);
  c.r_u = autoDeclareDouble(node, "baseline_mppi.r_u", c.r_u);
  autoDeclareArray(node, "baseline_mppi.r_dtau", c.r_dtau);
  c.terminal_mult = autoDeclareDouble(node, "baseline_mppi.terminal_mult", c.terminal_mult);
  c.x_wall = autoDeclareDouble(node, "baseline_mppi.x_wall", c.x_wall);
  c.wall_k = autoDeclareDouble(node, "baseline_mppi.wall_k", c.wall_k);
  c.wall_b = autoDeclareDouble(node, "baseline_mppi.wall_b", c.wall_b);
  c.preload_fx = autoDeclareDouble(node, "baseline_mppi.preload_fx", c.preload_fx);
  c.seed = static_cast<uint64_t>(
    autoDeclareInt64(node, "baseline_mppi.seed", static_cast<int64_t>(c.seed)));

  return c;
}

}  // namespace baseline_mppi
}  // namespace adhere_controller
