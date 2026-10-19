#ifndef ADHERE_CONTROLLER__ALGORITHMS__BASELINE_MPPI__BASELINEMPPICONTROLLER_H_
#define ADHERE_CONTROLLER__ALGORITHMS__BASELINE_MPPI__BASELINEMPPICONTROLLER_H_

#include <cuda_runtime.h>

#include <array>
#include <memory>

#include "adhere_controller/algorithms/baseline_mppi/BaselineCudaTypes.h"
#include "adhere_controller/algorithms/baseline_mppi/BaselineMppiConfig.h"
#include "adhere_controller/common/LoopProfiler.h"
#include "adhere_controller/common/mathTypes.h"
#include "adhere_controller/dynamics/FossenDynamics.h"
#include "adhere_controller/dynamics/ModelParams.h"

namespace adhere_controller
{
namespace baseline_mppi
{

// Pure-CUDA, outer-loop-only Baseline MPPI engine (CLAUDE.md Phase 4.1): owns every GPU/pinned
// buffer for its lifetime (allocated once in init(), freed in the destructor -- zero runtime
// allocations per solve(), per Sec.2.2.4). solve() is a synchronous call (internally does
// upload -> 4 kernel launches on a dedicated stream -> cudaStreamSynchronize -> pinned readback)
// -- it is meant to be invoked from AdhereController's already-asynchronous 25Hz outer-loop
// worker thread (adhere_controller.cpp::outerWorkerLoop()), never from the 500Hz physics thread.
// Deliberately returns raw allocated thruster commands (std::array<double,8>), NOT a
// NominalWrenchCmd -- Baseline MPPI does its own B^dagger allocation on-device (Kernel 4) and is
// explicitly simpler than the Phase-4-KAMR WBC/OSQP pipeline (algorithms/wbc/WbcAllocatorBase.h),
// which this class does not implement.
class BaselineMppiController
{
public:
  BaselineMppiController() = default;
  ~BaselineMppiController();

  BaselineMppiController(const BaselineMppiController &) = delete;
  BaselineMppiController & operator=(const BaselineMppiController &) = delete;

  // One-time GPU buffer allocation + __constant__ model upload + RNG seeding. b/b_pinv are the
  // same allocation::ThrusterAllocator-built matrices every FSMState already uses -- passed in
  // rather than rebuilt here so there is exactly one place (AdhereController::on_activate()) that
  // computes them from ModelParams.
  void init(const ModelParams & model_params, const Mat68 & b, const Mat86 & b_pinv,
    const BaselineMppiConfig & config);

  // Resets the persistent nominal wrench sequence to the hydrostatic-compensation wrench g(q_d)
  // (CLAUDE.md Phase 4.1 Sec.3.1): called by StateBaselineMppi::enter().
  void warmStart(const Vec3 & p_d, const Quat & q_d);

  // Synchronous GPU solve (see class doc comment on threading). pos_w/quat_w/vel_body/omega_body
  // are the current ActiveState snapshot; target_pos/target_quat are the live reference pose.
  // Returns the 8 clamped thruster commands u_0*.
  std::array<double, 8> solve(
    const Vec3 & pos_w, const Quat & quat_w, const Vec3 & vel_body, const Vec3 & omega_body,
    const Vec3 & target_pos, const Quat & target_quat);

  double lastSolveMs() const { return solve_profiler_.latestExecMs(); }

private:
  bool initialized_ = false;
  BaselineMppiConfig config_;
  CudaBaselineRuntimeConfig device_config_{};
  Mat86 b_pinv_;

  cudaStream_t stream_ = nullptr;
  void * d_rng_ = nullptr;
  float * d_tau_bar_ = nullptr;      // persistent nominal sequence, H*6
  float * d_tau_bar_new_ = nullptr;  // H*6
  float * d_eps_ = nullptr;          // K*H*6
  float * d_costs_ = nullptr;        // K
  float * d_weights_ = nullptr;      // K
  float * d_u0_ = nullptr;           // 8, device
  float * h_u0_pinned_ = nullptr;    // 8, cudaHostAlloc'd

  // Owns the SAME physical model warmStart() needs for its -g(q_d) hover-wrench reset (CLAUDE.md
  // Phase 4.1 Sec.3.1) -- built once in init() from the real ModelParams passed in, so warmStart()
  // never risks silently falling back to default-constructed physical parameters.
  std::unique_ptr<FossenDynamics> dynamics_;

  LoopProfiler solve_profiler_;

  void freeBuffers();
};

}  // namespace baseline_mppi
}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__ALGORITHMS__BASELINE_MPPI__BASELINEMPPICONTROLLER_H_
