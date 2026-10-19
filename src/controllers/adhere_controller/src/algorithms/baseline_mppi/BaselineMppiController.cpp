#include "adhere_controller/algorithms/baseline_mppi/BaselineMppiController.h"

#include <cmath>
#include <memory>
#include <vector>

#include "adhere_controller/algorithms/baseline_mppi/BaselineMppiKernels.h"
#include "adhere_controller/dynamics/FossenDynamics.h"

namespace adhere_controller
{
namespace baseline_mppi
{

namespace
{

CudaBaselineModelParams buildCudaBaselineModelParams(
  const ModelParams & mp, const Mat68 & b, const Mat86 & b_pinv)
{
  CudaBaselineModelParams out{};

  out.mass_diag[0] = static_cast<float>(mp.vehicle_mass + mp.dynamics.added_mass_lin.x());
  out.mass_diag[1] = static_cast<float>(mp.vehicle_mass + mp.dynamics.added_mass_lin.y());
  out.mass_diag[2] = static_cast<float>(mp.vehicle_mass + mp.dynamics.added_mass_lin.z());
  out.mass_diag[3] =
    static_cast<float>(mp.dynamics.inertia_diag.x() + mp.dynamics.added_mass_ang.x());
  out.mass_diag[4] =
    static_cast<float>(mp.dynamics.inertia_diag.y() + mp.dynamics.added_mass_ang.y());
  out.mass_diag[5] =
    static_cast<float>(mp.dynamics.inertia_diag.z() + mp.dynamics.added_mass_ang.z());
  for (int i = 0; i < 6; ++i) {
    out.mass_inv_diag[i] = 1.0f / out.mass_diag[i];
  }

  for (int i = 0; i < 3; ++i) {
    out.d_lin[i] = static_cast<float>(mp.D_lin(i));
    out.d_quad[i] = static_cast<float>(mp.D_quad(i));
    out.d_ang_lin[i] = static_cast<float>(mp.D_ang_lin(i));
    out.d_ang_quad[i] = static_cast<float>(mp.D_ang_quad(i));
    out.cob_offset_body[i] = static_cast<float>(mp.cob_offset_body(i));
  }

  out.buoyancy_full = static_cast<float>(mp.rho_water * mp.gravity * mp.displaced_volume);
  out.weight_full = static_cast<float>(mp.vehicle_mass * mp.gravity);

  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 8; ++c) {
      out.b_mat[r * 8 + c] = static_cast<float>(b(r, c));
    }
  }
  for (int r = 0; r < 8; ++r) {
    for (int c = 0; c < 6; ++c) {
      out.b_pinv[r * 6 + c] = static_cast<float>(b_pinv(r, c));
    }
  }
  for (int i = 0; i < 8; ++i) {
    out.u_min[i] = static_cast<float>(mp.thruster_min[i]);
    out.u_max[i] = static_cast<float>(mp.thruster_max[i]);
  }

  return out;
}

CudaBaselineRuntimeConfig buildDeviceConfig(const BaselineMppiConfig & c)
{
  CudaBaselineRuntimeConfig out{};
  out.num_rollouts = c.num_rollouts;
  out.horizon = c.horizon;
  out.dt = static_cast<float>(c.dt);
  out.lambda = static_cast<float>(c.lambda);
  for (int i = 0; i < 6; ++i) {
    out.sigma_tau[i] = static_cast<float>(c.sigma_tau[i]);
    out.q_vel[i] = static_cast<float>(c.q_vel[i]);
    out.r_dtau[i] = static_cast<float>(c.r_dtau[i]);
  }
  for (int i = 0; i < 3; ++i) {
    out.q_pos[i] = static_cast<float>(c.q_pos[i]);
  }
  out.q_quat = static_cast<float>(c.q_quat);
  out.r_u = static_cast<float>(c.r_u);
  out.terminal_mult = static_cast<float>(c.terminal_mult);
  out.x_wall = static_cast<float>(c.x_wall);
  out.wall_k = static_cast<float>(c.wall_k);
  out.wall_b = static_cast<float>(c.wall_b);
  out.preload_fx = static_cast<float>(c.preload_fx);
  out.seed = static_cast<unsigned long long>(c.seed);  // NOLINT(runtime/int)
  return out;
}

CudaState6Dof toCudaState(const Vec3 & p, const Quat & q, const Vec3 & v, const Vec3 & w)
{
  CudaState6Dof s{};
  s.p[0] = static_cast<float>(p.x());
  s.p[1] = static_cast<float>(p.y());
  s.p[2] = static_cast<float>(p.z());
  s.q[0] = static_cast<float>(q(0));
  s.q[1] = static_cast<float>(q(1));
  s.q[2] = static_cast<float>(q(2));
  s.q[3] = static_cast<float>(q(3));
  s.v[0] = static_cast<float>(v.x());
  s.v[1] = static_cast<float>(v.y());
  s.v[2] = static_cast<float>(v.z());
  s.w[0] = static_cast<float>(w.x());
  s.w[1] = static_cast<float>(w.y());
  s.w[2] = static_cast<float>(w.z());
  return s;
}

}  // namespace

BaselineMppiController::~BaselineMppiController()
{
  freeBuffers();
}

void BaselineMppiController::freeBuffers()
{
  if (d_tau_bar_) cudaFree(d_tau_bar_);
  if (d_tau_bar_new_) cudaFree(d_tau_bar_new_);
  if (d_eps_) cudaFree(d_eps_);
  if (d_costs_) cudaFree(d_costs_);
  if (d_weights_) cudaFree(d_weights_);
  if (d_u0_) cudaFree(d_u0_);
  if (h_u0_pinned_) cudaFreeHost(h_u0_pinned_);
  if (d_rng_) freeRngStates(d_rng_);
  if (stream_) cudaStreamDestroy(stream_);
  d_tau_bar_ = d_tau_bar_new_ = d_eps_ = d_costs_ = d_weights_ = d_u0_ = nullptr;
  h_u0_pinned_ = nullptr;
  d_rng_ = nullptr;
  stream_ = nullptr;
}

void BaselineMppiController::init(
  const ModelParams & model_params, const Mat68 & b, const Mat86 & b_pinv,
  const BaselineMppiConfig & config)
{
  if (initialized_) {
    freeBuffers();
  }

  config_ = config;
  b_pinv_ = b_pinv;
  device_config_ = buildDeviceConfig(config_);

  const CudaBaselineModelParams cuda_params = buildCudaBaselineModelParams(model_params, b, b_pinv);
  uploadModelParams(cuda_params);

  cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);

  const int k = config_.num_rollouts;
  const int h = config_.horizon;
  cudaMalloc(reinterpret_cast<void **>(&d_tau_bar_), sizeof(float) * h * 6);
  cudaMalloc(reinterpret_cast<void **>(&d_tau_bar_new_), sizeof(float) * h * 6);
  cudaMalloc(reinterpret_cast<void **>(&d_eps_), sizeof(float) * static_cast<size_t>(k) * h * 6);
  cudaMalloc(reinterpret_cast<void **>(&d_costs_), sizeof(float) * k);
  cudaMalloc(reinterpret_cast<void **>(&d_weights_), sizeof(float) * k);
  cudaMalloc(reinterpret_cast<void **>(&d_u0_), sizeof(float) * 8);
  cudaHostAlloc(reinterpret_cast<void **>(&h_u0_pinned_), sizeof(float) * 8, cudaHostAllocDefault);

  d_rng_ = allocRngStates(k);
  initRngStates(d_rng_, k, config_.seed);

  cudaMemset(d_tau_bar_, 0, sizeof(float) * h * 6);

  dynamics_ = std::make_unique<FossenDynamics>(model_params);

  initialized_ = true;
}

void BaselineMppiController::warmStart(const Vec3 & p_d, const Quat & q_d)
{
  (void)p_d;
  if (!initialized_ || !dynamics_) {
    return;
  }

  // Resets the nominal sequence to the constant hydrostatic-compensation wrench -g(q_d) (CLAUDE.md
  // Phase 4.1 Sec.3.1): the feedforward sign convention FossenDynamics::computeRestoringWrench()
  // already documents (M*nu_dot + C*nu + D*nu + g(q) = tau -> a hovering controller commands
  // tau = -g(q)), evaluated with zero-velocity assumption (enter() latches the current pose as
  // target, so position/velocity error is ~0 at the moment this is called).
  const Vec6 g = dynamics_->computeRestoringWrench(q_d);

  std::vector<float> tau0(6);
  for (int i = 0; i < 6; ++i) {
    tau0[i] = static_cast<float>(-g(i));
  }
  std::vector<float> tau_bar(static_cast<size_t>(config_.horizon) * 6);
  for (int t = 0; t < config_.horizon; ++t) {
    for (int i = 0; i < 6; ++i) {
      tau_bar[t * 6 + i] = tau0[i];
    }
  }
  cudaMemcpyAsync(
    d_tau_bar_, tau_bar.data(), tau_bar.size() * sizeof(float), cudaMemcpyHostToDevice, stream_);
  cudaStreamSynchronize(stream_);
}

std::array<double, 8> BaselineMppiController::solve(
  const Vec3 & pos_w, const Quat & quat_w, const Vec3 & vel_body, const Vec3 & omega_body,
  const Vec3 & target_pos, const Quat & target_quat)
{
  std::array<double, 8> u_star{};
  u_star.fill(0.0);
  if (!initialized_) {
    return u_star;
  }

  solve_profiler_.beginExec();

  const CudaState6Dof x0 = toCudaState(pos_w, quat_w, vel_body, omega_body);
  const CudaState6Dof target =
    toCudaState(target_pos, target_quat, Vec3::Zero(), Vec3::Zero());

  launchRollout(device_config_, x0, target, d_tau_bar_, d_eps_, d_costs_, d_rng_, stream_);
  launchSoftmaxWeights(
    device_config_.num_rollouts, device_config_.lambda, d_costs_, d_weights_, stream_);
  launchUpdateNominal(
    device_config_, d_weights_, d_eps_, d_tau_bar_, d_tau_bar_new_, stream_);
  launchFinalize(device_config_, d_tau_bar_new_, d_tau_bar_, d_u0_, stream_);

  cudaMemcpyAsync(
    h_u0_pinned_, d_u0_, sizeof(float) * 8, cudaMemcpyDeviceToHost, stream_);
  cudaStreamSynchronize(stream_);

  for (int i = 0; i < 8; ++i) {
    const float v = h_u0_pinned_[i];
    u_star[i] = std::isfinite(v) ? static_cast<double>(v) : 0.0;
  }

  solve_profiler_.endExec();
  return u_star;
}

}  // namespace baseline_mppi
}  // namespace adhere_controller
