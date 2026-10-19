#include "adhere_controller/algorithms/baseline_mppi/BaselineMppiKernels.h"

#include <curand_kernel.h>

namespace adhere_controller
{
namespace baseline_mppi
{

namespace
{

// Uploaded once by uploadModelParams() (CLAUDE.md Phase 4.1 Sec.2.1.2): the single GPU-resident
// copy of the float-cast physical model, read by every kernel below.
__constant__ CudaBaselineModelParams d_baseline_model_params;

constexpr int kStateDim = 6;   // [Fx,Fy,Fz,Tx,Ty,Tz] wrench / [vx,vy,vz,wx,wy,wz] twist DOF count
constexpr int kNumThrusters = 8;
constexpr float kWallMargin = 0.10f;  // m, "near the wall" preload-bias activation band

// ---------------------------------------------------------------------------------------------
// Small device-side math helpers (quaternion / vector), float32, mirroring common/mathTools.h's
// double-precision Hamilton (w,x,y,z) convention exactly so host (FossenDynamics) and device
// (this file) agree bit-for-bit on sign conventions.
// ---------------------------------------------------------------------------------------------

__device__ __forceinline__ void quatNormalize(float q[4])
{
  const float n = rsqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] + 1e-24f);
  q[0] *= n;
  q[1] *= n;
  q[2] *= n;
  q[3] *= n;
}

__device__ __forceinline__ void quatMul(const float a[4], const float b[4], float out[4])
{
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

// R(q) * v: body -> world.
__device__ __forceinline__ void rotateByQuat(const float q[4], const float v[3], float out[3])
{
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  out[0] = (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - w * z) * v[1] + 2 * (x * z + w * y) * v[2];
  out[1] = 2 * (x * y + w * z) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z - w * x) * v[2];
  out[2] = 2 * (x * z - w * y) * v[0] + 2 * (y * z + w * x) * v[1] + (1 - 2 * (x * x + y * y)) * v[2];
}

// R(q)^T * v: world -> body.
__device__ __forceinline__ void rotateByQuatInv(const float q[4], const float v[3], float out[3])
{
  const float qi[4] = {q[0], -q[1], -q[2], -q[3]};
  rotateByQuat(qi, v, out);
}

// ---------------------------------------------------------------------------------------------
// Diagonal Fossen 6-DOF dynamics (float mirror of dynamics::FossenDynamics, CLAUDE.md Phase
// 3.7) + a unilateral Kelvin-Voigt-style wall contact penalty (CLAUDE.md Sec.4.2) so a rollout
// cannot tunnel through the wall at x <= x_wall. tau[6] is the commanded body wrench (preload
// bias already folded in by the caller); the contact reaction is added on top here, purely for
// integration -- cost evaluation upstream uses the caller's tau, not this augmented value.
// ---------------------------------------------------------------------------------------------

struct StateDeriv
{
  float pdot[3];
  float qdot[4];
  float vdot[3];
  float wdot[3];
};

__device__ __forceinline__ StateDeriv computeDeriv(
  const CudaState6Dof & s, const float tau[kStateDim], const CudaBaselineRuntimeConfig & cfg)
{
  const CudaBaselineModelParams & mp = d_baseline_model_params;

  const float vx = s.v[0], vy = s.v[1], vz = s.v[2];
  const float wx = s.w[0], wy = s.w[1], wz = s.w[2];
  const float m1 = mp.mass_diag[0], m2 = mp.mass_diag[1], m3 = mp.mass_diag[2];
  const float m4 = mp.mass_diag[3], m5 = mp.mass_diag[4], m6 = mp.mass_diag[5];

  // Closed-form skew-symmetric rigid-body+added-mass Coriolis-centripetal term C(nu)*nu
  // (Fossen eq. 6.43 for a diagonal mass matrix), algebraically simplified from the same 6x6
  // matrix FossenDynamics::coriolisWrench() builds explicitly.
  float c_nu[kStateDim];
  c_nu[0] = m3 * vz * wy - m2 * vy * wz;
  c_nu[1] = -m3 * vz * wx + m1 * vx * wz;
  c_nu[2] = m2 * vy * wx - m1 * vx * wy;
  c_nu[3] = (m3 - m2) * vy * vz + (m6 - m5) * wy * wz;
  c_nu[4] = (m1 - m3) * vx * vz + (m4 - m6) * wx * wz;
  c_nu[5] = (m2 - m1) * vx * vy + (m5 - m4) * wx * wy;

  float d_nu[kStateDim];
  d_nu[0] = mp.d_lin[0] * vx + mp.d_quad[0] * vx * fabsf(vx);
  d_nu[1] = mp.d_lin[1] * vy + mp.d_quad[1] * vy * fabsf(vy);
  d_nu[2] = mp.d_lin[2] * vz + mp.d_quad[2] * vz * fabsf(vz);
  d_nu[3] = mp.d_ang_lin[0] * wx + mp.d_ang_quad[0] * wx * fabsf(wx);
  d_nu[4] = mp.d_ang_lin[1] * wy + mp.d_ang_quad[1] * wy * fabsf(wy);
  d_nu[5] = mp.d_ang_lin[2] * wz + mp.d_ang_quad[2] * wz * fabsf(wz);

  const float f_buoy_net_world[3] = {0.0f, 0.0f, mp.buoyancy_full - mp.weight_full};
  const float f_buoy_full_world[3] = {0.0f, 0.0f, mp.buoyancy_full};
  float g_lin[3];
  rotateByQuatInv(s.q, f_buoy_net_world, g_lin);
  float r_full_body[3];
  rotateByQuatInv(s.q, f_buoy_full_world, r_full_body);
  const float g_ang[3] = {
    mp.cob_offset_body[1] * r_full_body[2] - mp.cob_offset_body[2] * r_full_body[1],
    mp.cob_offset_body[2] * r_full_body[0] - mp.cob_offset_body[0] * r_full_body[2],
    mp.cob_offset_body[0] * r_full_body[1] - mp.cob_offset_body[1] * r_full_body[0]};

  // Unilateral wall contact penalty (rollout-only, not a thruster force): opposes further -X
  // penetration past x_wall with a stiff spring-damper, world-frame, rotated into body frame.
  float tau_contact[kStateDim] = {0, 0, 0, 0, 0, 0};
  if (s.p[0] <= cfg.x_wall) {
    const float penetration = cfg.x_wall - s.p[0];
    float v_world[3];
    rotateByQuat(s.q, s.v, v_world);
    const float lambda_n = fmaxf(0.0f, cfg.wall_k * penetration - cfg.wall_b * v_world[0]);
    const float f_world[3] = {lambda_n, 0.0f, 0.0f};
    float f_body[3];
    rotateByQuatInv(s.q, f_world, f_body);
    tau_contact[0] = f_body[0];
    tau_contact[1] = f_body[1];
    tau_contact[2] = f_body[2];
  }

  float nu_dot[kStateDim];
#pragma unroll
  for (int i = 0; i < kStateDim; ++i) {
    nu_dot[i] = mp.mass_inv_diag[i] *
      (tau[i] + tau_contact[i] - c_nu[i] - d_nu[i] -
       (i < 3 ? g_lin[i] : g_ang[i - 3]));
  }

  StateDeriv d;
  rotateByQuat(s.q, s.v, d.pdot);
  const float half_w[4] = {0.0f, 0.5f * wx, 0.5f * wy, 0.5f * wz};
  quatMul(s.q, half_w, d.qdot);
  d.vdot[0] = nu_dot[0];
  d.vdot[1] = nu_dot[1];
  d.vdot[2] = nu_dot[2];
  d.wdot[0] = nu_dot[3];
  d.wdot[1] = nu_dot[4];
  d.wdot[2] = nu_dot[5];
  return d;
}

// RK2 (explicit midpoint) integration of one dt step under a piecewise-constant commanded wrench
// tau (CLAUDE.md Phase 4.1 Sec.2.2.3): the wall-contact reaction in computeDeriv() is re-evaluated
// at both the start and the midpoint state, so a rollout that penetrates mid-step still feels the
// corrective spring-damper.
__device__ __forceinline__ CudaState6Dof stepBaselineFossen6Dof(
  const CudaState6Dof & s, const float tau[kStateDim], const CudaBaselineRuntimeConfig & cfg,
  float dt)
{
  const StateDeriv k1 = computeDeriv(s, tau, cfg);

  CudaState6Dof mid = s;
  mid.p[0] += 0.5f * dt * k1.pdot[0];
  mid.p[1] += 0.5f * dt * k1.pdot[1];
  mid.p[2] += 0.5f * dt * k1.pdot[2];
  mid.q[0] += 0.5f * dt * k1.qdot[0];
  mid.q[1] += 0.5f * dt * k1.qdot[1];
  mid.q[2] += 0.5f * dt * k1.qdot[2];
  mid.q[3] += 0.5f * dt * k1.qdot[3];
  quatNormalize(mid.q);
  mid.v[0] += 0.5f * dt * k1.vdot[0];
  mid.v[1] += 0.5f * dt * k1.vdot[1];
  mid.v[2] += 0.5f * dt * k1.vdot[2];
  mid.w[0] += 0.5f * dt * k1.wdot[0];
  mid.w[1] += 0.5f * dt * k1.wdot[1];
  mid.w[2] += 0.5f * dt * k1.wdot[2];

  const StateDeriv k2 = computeDeriv(mid, tau, cfg);

  CudaState6Dof out = s;
  out.p[0] += dt * k2.pdot[0];
  out.p[1] += dt * k2.pdot[1];
  out.p[2] += dt * k2.pdot[2];
  out.q[0] += dt * k2.qdot[0];
  out.q[1] += dt * k2.qdot[1];
  out.q[2] += dt * k2.qdot[2];
  out.q[3] += dt * k2.qdot[3];
  quatNormalize(out.q);
  out.v[0] += dt * k2.vdot[0];
  out.v[1] += dt * k2.vdot[1];
  out.v[2] += dt * k2.vdot[2];
  out.w[0] += dt * k2.wdot[0];
  out.w[1] += dt * k2.wdot[1];
  out.w[2] += dt * k2.wdot[2];
  return out;
}

// B^dagger * tau (8x6 row-major Bpinv) -> clamp(u, u_min, u_max).
__device__ __forceinline__ void allocateAndClamp(const float tau[kStateDim], float u[kNumThrusters])
{
  const CudaBaselineModelParams & mp = d_baseline_model_params;
#pragma unroll
  for (int i = 0; i < kNumThrusters; ++i) {
    float acc = 0.0f;
#pragma unroll
    for (int j = 0; j < kStateDim; ++j) {
      acc += mp.b_pinv[i * kStateDim + j] * tau[j];
    }
    u[i] = fminf(fmaxf(acc, mp.u_min[i]), mp.u_max[i]);
  }
}

// B * u (6x8 row-major B) -> realized wrench.
__device__ __forceinline__ void realizeWrench(const float u[kNumThrusters], float tau_real[kStateDim])
{
  const CudaBaselineModelParams & mp = d_baseline_model_params;
#pragma unroll
  for (int r = 0; r < kStateDim; ++r) {
    float acc = 0.0f;
#pragma unroll
    for (int c = 0; c < kNumThrusters; ++c) {
      acc += mp.b_mat[r * kNumThrusters + c] * u[c];
    }
    tau_real[r] = acc;
  }
}

__device__ __forceinline__ float squaredWeightedNorm3(
  const float e[3], const float w[3])
{
  return w[0] * e[0] * e[0] + w[1] * e[1] * e[1] + w[2] * e[2] * e[2];
}

// ---------------------------------------------------------------------------------------------
// Kernel 1: one thread per rollout.
// ---------------------------------------------------------------------------------------------
__global__ void rolloutBaselineMppiKernel(
  CudaBaselineRuntimeConfig cfg, CudaState6Dof x0, CudaState6Dof target, const float * tau_bar,
  float * eps, float * costs, curandStatePhilox4_32_10_t * rng)
{
  const int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= cfg.num_rollouts) {
    return;
  }

  curandStatePhilox4_32_10_t local_rng = rng[k];
  CudaState6Dof s = x0;
  float tau_prev[kStateDim] = {0, 0, 0, 0, 0, 0};
  float cost = 0.0f;

  for (int t = 0; t < cfg.horizon; ++t) {
    float e[kStateDim];
    e[0] = curand_normal(&local_rng) * cfg.sigma_tau[0];
    e[1] = curand_normal(&local_rng) * cfg.sigma_tau[1];
    e[2] = curand_normal(&local_rng) * cfg.sigma_tau[2];
    e[3] = curand_normal(&local_rng) * cfg.sigma_tau[3];
    e[4] = curand_normal(&local_rng) * cfg.sigma_tau[4];
    e[5] = curand_normal(&local_rng) * cfg.sigma_tau[5];

    const float * tau_bar_t = tau_bar + t * kStateDim;
    float tau_cmd[kStateDim];
#pragma unroll
    for (int i = 0; i < kStateDim; ++i) {
      tau_cmd[i] = tau_bar_t[i] + e[i];
    }

    float u[kNumThrusters];
    allocateAndClamp(tau_cmd, u);
    float tau_real[kStateDim];
    realizeWrench(u, tau_real);
    if (s.p[0] <= cfg.x_wall + kWallMargin) {
      tau_real[0] -= cfg.preload_fx;
    }

    s = stepBaselineFossen6Dof(s, tau_real, cfg, cfg.dt);

    // Numerical fuse (CLAUDE.md Phase 4.1 Stage 3): explicit RK2 with a stiff wall-contact
    // spring-damper (wall_k/wall_b) and quadratic drag can, for a small fraction of unlucky
    // Gaussian exploration samples, diverge within the 30-step/1.2s horizon. Clamping velocity
    // magnitudes to a generous bound well above anything physically meaningful for this ROV
    // prevents that divergence from ever reaching Inf/NaN, so a single bad rollout can never
    // poison the softmax/weighted-update reduction shared across all K rollouts (see the
    // finite-cost sentinel below for the other half of this safety net).
#pragma unroll
    for (int i = 0; i < 3; ++i) {
      s.v[i] = fminf(fmaxf(s.v[i], -50.0f), 50.0f);
      s.w[i] = fminf(fmaxf(s.w[i], -50.0f), 50.0f);
    }

    // Quaternion-based running cost (CLAUDE.md Phase 4.1 Sec.2.2.2).
    const float e_p[3] = {s.p[0] - target.p[0], s.p[1] - target.p[1], s.p[2] - target.p[2]};
    float q_inv[4] = {s.q[0], -s.q[1], -s.q[2], -s.q[3]};
    float q_e[4];
    quatMul(q_inv, target.q, q_e);
    const float sign = q_e[0] < 0.0f ? -1.0f : 1.0f;
    const float e_r[3] = {2.0f * sign * q_e[1], 2.0f * sign * q_e[2], 2.0f * sign * q_e[3]};
    const float e_v[3] = {s.v[0] - target.v[0], s.v[1] - target.v[1], s.v[2] - target.v[2]};
    const float e_w[3] = {s.w[0] - target.w[0], s.w[1] - target.w[1], s.w[2] - target.w[2]};

    float u_sq = 0.0f;
#pragma unroll
    for (int i = 0; i < kNumThrusters; ++i) {
      u_sq += u[i] * u[i];
    }

    float dtau_cost = 0.0f;
    if (t > 0) {
#pragma unroll
      for (int i = 0; i < kStateDim; ++i) {
        const float d = tau_cmd[i] - tau_prev[i];
        dtau_cost += cfg.r_dtau[i] * d * d;
      }
    }

    float cross_term = 0.0f;
#pragma unroll
    for (int i = 0; i < kStateDim; ++i) {
      const float sigma_sq = cfg.sigma_tau[i] * cfg.sigma_tau[i] + 1e-9f;
      cross_term += cfg.lambda * tau_bar_t[i] * e[i] / sigma_sq;
    }

    float step_cost = squaredWeightedNorm3(e_p, cfg.q_pos) + cfg.q_quat * (e_r[0] * e_r[0] + e_r[1] * e_r[1] + e_r[2] * e_r[2]) +
      squaredWeightedNorm3(e_v, cfg.q_vel) + squaredWeightedNorm3(e_w, cfg.q_vel + 3) +
      cfg.r_u * u_sq + dtau_cost + cross_term;

    if (t == cfg.horizon - 1) {
      step_cost = (squaredWeightedNorm3(e_p, cfg.q_pos) +
                   cfg.q_quat * (e_r[0] * e_r[0] + e_r[1] * e_r[1] + e_r[2] * e_r[2]) +
                   squaredWeightedNorm3(e_v, cfg.q_vel) +
                   squaredWeightedNorm3(e_w, cfg.q_vel + 3)) *
          cfg.terminal_mult +
        cfg.r_u * u_sq + dtau_cost + cross_term;
    }

    cost += step_cost;

#pragma unroll
    for (int i = 0; i < kStateDim; ++i) {
      eps[(static_cast<long long>(k) * cfg.horizon + t) * kStateDim + i] = e[i];
      tau_prev[i] = tau_cmd[i];
    }
  }

  // Finite-cost sentinel (CLAUDE.md Phase 4.1 Stage 3): belt-and-suspenders alongside the
  // velocity clamp above -- isfinite() guards against any remaining NaN/Inf path (e.g. a
  // genuinely degenerate quaternion) ever reaching softmaxWeightsKernel, where a single
  // non-finite cost would otherwise poison the shared eta/weighted-sum reduction for ALL K
  // rollouts. A large-but-finite sentinel instead earns this rollout a ~zero softmax weight.
  costs[k] = isfinite(cost) ? cost : 1.0e12f;
  rng[k] = local_rng;
}

// ---------------------------------------------------------------------------------------------
// Kernel 2: single block, two-pass (min, then sum-of-exp) shared-mem + warp-shuffle reduction.
// ---------------------------------------------------------------------------------------------
__device__ __forceinline__ float warpReduceMin(float v)
{
  for (int offset = 16; offset > 0; offset >>= 1) {
    v = fminf(v, __shfl_down_sync(0xffffffffu, v, offset));
  }
  return v;
}
__device__ __forceinline__ float warpReduceSum(float v)
{
  for (int offset = 16; offset > 0; offset >>= 1) {
    v += __shfl_down_sync(0xffffffffu, v, offset);
  }
  return v;
}

__global__ void softmaxWeightsKernel(
  int num_rollouts, float lambda, const float * costs, float * weights)
{
  extern __shared__ float smem[];
  const int tid = threadIdx.x;
  const int warp = tid / 32;
  const int lane = tid % 32;
  const int num_warps = (blockDim.x + 31) / 32;

  float local_min = 3.0e38f;
  for (int k = tid; k < num_rollouts; k += blockDim.x) {
    local_min = fminf(local_min, costs[k]);
  }
  local_min = warpReduceMin(local_min);
  if (lane == 0) {
    smem[warp] = local_min;
  }
  __syncthreads();
  float rho;
  if (tid == 0) {
    rho = smem[0];
    for (int w = 1; w < num_warps; ++w) {
      rho = fminf(rho, smem[w]);
    }
    smem[0] = rho;
  }
  __syncthreads();
  rho = smem[0];
  __syncthreads();

  float local_sum = 0.0f;
  float min_diff = 3.0e38f;
  for (int k = tid; k < num_rollouts; k += blockDim.x) {
    const float diff = costs[k] - rho;
    min_diff = fminf(min_diff, diff);
    local_sum += expf(-diff / lambda);
  }
  local_sum = warpReduceSum(local_sum);
  if (lane == 0) {
    smem[warp] = local_sum;
  }
  __syncthreads();
  float eta;
  if (tid == 0) {
    eta = 0.0f;
    for (int w = 0; w < num_warps; ++w) {
      eta += smem[w];
    }
    smem[0] = fmaxf(eta, 1e-30f);
  }
  __syncthreads();
  eta = smem[0];

  for (int k = tid; k < num_rollouts; k += blockDim.x) {
    weights[k] = expf(-(costs[k] - rho) / lambda) / eta;
  }
}

// ---------------------------------------------------------------------------------------------
// Kernel 3: one block per horizon step t, grid-stride weighted-sum reduction over K rollouts.
// ---------------------------------------------------------------------------------------------
__global__ void updateNominalKernel(
  int num_rollouts, int horizon, const float * weights, const float * eps, const float * tau_bar,
  float * tau_bar_new)
{
  extern __shared__ float smem[];
  const int t = blockIdx.x;
  const int tid = threadIdx.x;
  const int warp = tid / 32;
  const int lane = tid % 32;
  const int num_warps = (blockDim.x + 31) / 32;

  float weighted[kStateDim] = {0, 0, 0, 0, 0, 0};
  for (int k = tid; k < num_rollouts; k += blockDim.x) {
    const float w = weights[k];
    const float * e = eps + (static_cast<long long>(k) * horizon + t) * kStateDim;
#pragma unroll
    for (int i = 0; i < kStateDim; ++i) {
      weighted[i] += w * e[i];
    }
  }

#pragma unroll
  for (int i = 0; i < kStateDim; ++i) {
    float v = warpReduceSum(weighted[i]);
    if (lane == 0) {
      smem[warp] = v;
    }
    __syncthreads();
    if (tid == 0) {
      float total = 0.0f;
      for (int w = 0; w < num_warps; ++w) {
        total += smem[w];
      }
      tau_bar_new[t * kStateDim + i] = tau_bar[t * kStateDim + i] + total;
    }
    __syncthreads();
  }
}

// ---------------------------------------------------------------------------------------------
// Kernel 4: single block, H threads. 3-tap smoothing + warm-start shift + u_0* finalize.
// ---------------------------------------------------------------------------------------------
__global__ void finalizeNominalKernel(
  int horizon, const float * tau_bar_new, float * tau_bar, float * u0)
{
  extern __shared__ float smooth[];  // horizon * kStateDim
  const int t = threadIdx.x;

  if (t < horizon) {
    const int tm = t > 0 ? t - 1 : 0;
    const int tp = t < horizon - 1 ? t + 1 : horizon - 1;
#pragma unroll
    for (int i = 0; i < kStateDim; ++i) {
      smooth[t * kStateDim + i] = 0.25f * tau_bar_new[tm * kStateDim + i] +
        0.5f * tau_bar_new[t * kStateDim + i] + 0.25f * tau_bar_new[tp * kStateDim + i];
    }
  }
  __syncthreads();

  // Warm-start shift (CLAUDE.md Phase 4.1 Sec.2.2.3): tau_bar[t] <- smooth[t+1], last step repeats.
  if (t < horizon) {
    const int src = t < horizon - 1 ? t + 1 : horizon - 1;
#pragma unroll
    for (int i = 0; i < kStateDim; ++i) {
      tau_bar[t * kStateDim + i] = smooth[src * kStateDim + i];
    }
  }

  if (t < kNumThrusters) {
    float tau0[kStateDim];
#pragma unroll
    for (int i = 0; i < kStateDim; ++i) {
      tau0[i] = smooth[i];
    }
    const CudaBaselineModelParams & mp = d_baseline_model_params;
    float acc = 0.0f;
#pragma unroll
    for (int j = 0; j < kStateDim; ++j) {
      acc += mp.b_pinv[t * kStateDim + j] * tau0[j];
    }
    u0[t] = fminf(fmaxf(acc, mp.u_min[t]), mp.u_max[t]);
  }
}

__global__ void initRngStatesKernel(curandStatePhilox4_32_10_t * rng, int num_rollouts, unsigned long long seed)
{
  const int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= num_rollouts) {
    return;
  }
  curand_init(seed, static_cast<unsigned long long>(k), 0ULL, &rng[k]);
}

}  // namespace

void uploadModelParams(const CudaBaselineModelParams & params)
{
  cudaMemcpyToSymbol(d_baseline_model_params, &params, sizeof(CudaBaselineModelParams));
}

void * allocRngStates(int num_rollouts)
{
  curandStatePhilox4_32_10_t * ptr = nullptr;
  cudaMalloc(reinterpret_cast<void **>(&ptr), sizeof(curandStatePhilox4_32_10_t) * num_rollouts);
  return ptr;
}

void freeRngStates(void * d_rng)
{
  if (d_rng != nullptr) {
    cudaFree(d_rng);
  }
}

void initRngStates(void * d_rng, int num_rollouts, unsigned long long seed)
{
  const int block = 256;
  const int grid = (num_rollouts + block - 1) / block;
  initRngStatesKernel<<<grid, block>>>(
    static_cast<curandStatePhilox4_32_10_t *>(d_rng), num_rollouts, seed);
  cudaDeviceSynchronize();
}

void launchRollout(
  const CudaBaselineRuntimeConfig & cfg, const CudaState6Dof & x0, const CudaState6Dof & target,
  const float * d_tau_bar, float * d_eps, float * d_costs, void * d_rng, cudaStream_t stream)
{
  const int block = 256;
  const int grid = (cfg.num_rollouts + block - 1) / block;
  rolloutBaselineMppiKernel<<<grid, block, 0, stream>>>(
    cfg, x0, target, d_tau_bar, d_eps, d_costs, static_cast<curandStatePhilox4_32_10_t *>(d_rng));
}

void launchSoftmaxWeights(
  int num_rollouts, float lambda, const float * d_costs, float * d_weights, cudaStream_t stream)
{
  const int block = 256;
  const int num_warps = (block + 31) / 32;
  softmaxWeightsKernel<<<1, block, num_warps * sizeof(float), stream>>>(
    num_rollouts, lambda, d_costs, d_weights);
}

void launchUpdateNominal(
  const CudaBaselineRuntimeConfig & cfg, const float * d_weights, const float * d_eps,
  const float * d_tau_bar, float * d_tau_bar_new, cudaStream_t stream)
{
  const int block = 256;
  const int num_warps = (block + 31) / 32;
  updateNominalKernel<<<cfg.horizon, block, num_warps * sizeof(float), stream>>>(
    cfg.num_rollouts, cfg.horizon, d_weights, d_eps, d_tau_bar, d_tau_bar_new);
}

void launchFinalize(
  const CudaBaselineRuntimeConfig & cfg, const float * d_tau_bar_new, float * d_tau_bar,
  float * d_u0, cudaStream_t stream)
{
  const int block = cfg.horizon > kNumThrusters ? cfg.horizon : kNumThrusters;
  finalizeNominalKernel<<<1, block, cfg.horizon * kStateDim * sizeof(float), stream>>>(
    cfg.horizon, d_tau_bar_new, d_tau_bar, d_u0);
}

}  // namespace baseline_mppi
}  // namespace adhere_controller
