#ifndef ADHERE_CONTROLLER__ALGORITHMS__BASELINE_MPPI__BASELINEMPPIKERNELS_H_
#define ADHERE_CONTROLLER__ALGORITHMS__BASELINE_MPPI__BASELINEMPPIKERNELS_H_

#include <cuda_runtime.h>

#include "adhere_controller/algorithms/baseline_mppi/BaselineCudaTypes.h"

// Host-callable wrapper declarations (CLAUDE.md Phase 4.1 Sec.2.2): the ONLY header shared
// between BaselineMppiController.cpp (plain g++, host-side buffer/stream ownership) and
// baseline_mppi_kernels.cu (nvcc, kernel definitions + these wrapper bodies). Deliberately
// exposes zero CUDA-kernel-launch syntax and zero curand types to the host compiler -- the
// Philox RNG state array is an opaque device pointer (void*) here; its real type
// (curandStatePhilox4_32_10_t[K]) and <curand_kernel.h> stay entirely inside the .cu file.
namespace adhere_controller
{
namespace baseline_mppi
{

// Uploads params to GPU __constant__ memory (d_baseline_model_params). Called once, synchronously,
// from BaselineMppiController::init().
void uploadModelParams(const CudaBaselineModelParams & params);

void * allocRngStates(int num_rollouts);
void freeRngStates(void * d_rng);
// One-time Philox state seeding (one subsequence per rollout index), synchronous.
void initRngStates(
  void * d_rng, int num_rollouts, unsigned long long seed);  // NOLINT(runtime/int)

// Kernel 1: one thread per rollout k, RK2-integrates H steps of stepBaselineFossen6Dof, samples
// eps_{k,t} ~ N(0, diag(sigma_tau)) from d_rng, writes eps_{k,t} to d_eps (K*H*6, row-major
// [k*H*6 + t*6 + dof]) and the total trajectory cost S_k to d_costs[k].
void launchRollout(
  const CudaBaselineRuntimeConfig & cfg, const CudaState6Dof & x0, const CudaState6Dof & target,
  const float * d_tau_bar, float * d_eps, float * d_costs, void * d_rng, cudaStream_t stream);

// Kernel 2: single block, two-pass shared-mem + warp-shuffle reduction over d_costs[0..K-1] ->
// rho=min(S), eta=sum(exp(-(S-rho)/lambda)), writes normalized weights w_k to d_weights[k].
void launchSoftmaxWeights(
  int num_rollouts, float lambda, const float * d_costs, float * d_weights, cudaStream_t stream);

// Kernel 3: one block per horizon step t in [0,H), grid-stride + shared-mem reduction of
// sum_k w_k * eps_{k,t} (6-dim), writes tau_bar_new[t] = tau_bar[t] + that weighted sum.
void launchUpdateNominal(
  const CudaBaselineRuntimeConfig & cfg, const float * d_weights, const float * d_eps,
  const float * d_tau_bar, float * d_tau_bar_new, cudaStream_t stream);

// Kernel 4: single block, H threads. 3-tap-smooths tau_bar_new -> writes the warm-started
// (shifted-by-1) sequence back into tau_bar for the NEXT solve() call, and writes
// u_0* = clamp(B^dagger * tau_bar_new[0], u_min, u_max) into d_u0 (device, 8 floats) for the
// caller to cudaMemcpyAsync back to pinned host memory.
void launchFinalize(
  const CudaBaselineRuntimeConfig & cfg, const float * d_tau_bar_new, float * d_tau_bar,
  float * d_u0, cudaStream_t stream);

}  // namespace baseline_mppi
}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__ALGORITHMS__BASELINE_MPPI__BASELINEMPPIKERNELS_H_
