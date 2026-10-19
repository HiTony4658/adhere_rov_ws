#ifndef ADHERE_CONTROLLER__ALGORITHMS__BASELINE_MPPI__BASELINECUDATYPES_H_
#define ADHERE_CONTROLLER__ALGORITHMS__BASELINE_MPPI__BASELINECUDATYPES_H_

// Zero-Eigen/zero-ROS POD float structs (CLAUDE.md Phase 4.1 Stage 2.1.2): this header is
// included by both baseline_mppi_kernels.cu (nvcc) and BaselineMppiController.cpp (host g++), so
// it must stay free of Eigen/rclcpp to keep nvcc's compile fast and avoid dragging Eigen's
// expression-template machinery into device code. dynamics/ModelParams.h (Eigen-based, double)
// remains the single physical-truth source; CudaBaselineModelParams::fromHostParams() (defined in
// BaselineMppiController.cpp, which *does* see Eigen) is the one-time float-cast bridge, uploaded
// to GPU __constant__ memory exactly once in BaselineMppiController::init().

namespace adhere_controller
{
namespace baseline_mppi
{

// 6-DOF rigid-body state used by the device-side rollout integrator. Position/orientation are
// world-frame; v/w are body-frame linear/angular velocity (matches CtrlInterfaces::ActiveState's
// convention, CLAUDE.md Phase 3.5).
struct CudaState6Dof
{
  float p[3];   // world position [x, y, z]
  float q[4];   // world orientation, Hamilton (w, x, y, z)
  float v[3];   // body-frame linear velocity
  float w[3];   // body-frame angular velocity
};

// Diagonal Fossen rigid-body + added-mass + damping + hydrostatic model (float mirror of
// dynamics::FossenDynamics, CLAUDE.md Phase 3.7/Sec.5), plus the 6x8 wrench allocation matrix B
// and its 8x6 damped pseudoinverse B^dagger (allocation::ThrusterAllocator, CLAUDE.md Sec.4.3) --
// both built once on the host from ModelParams' thruster geometry and uploaded alongside, so the
// GPU rollout never recomputes allocation. Row-major flattening: B[row*8+col] (6x8),
// Bpinv[row*6+col] (8x6).
struct CudaBaselineModelParams
{
  float mass_diag[6];        // [m+addedX, m+addedY, m+addedZ, Ixx+addedRoll, Iyy+addedPitch,
                              // Izz+addedYaw]
  float mass_inv_diag[6];    // 1 / mass_diag[i]
  float d_lin[3];
  float d_quad[3];
  float d_ang_lin[3];
  float d_ang_quad[3];
  float buoyancy_full;       // rho_water * gravity * displaced_volume (N)
  float weight_full;         // vehicle_mass * gravity (N)
  float cob_offset_body[3];
  float b_mat[48];           // 6x8, row-major
  float b_pinv[48];          // 8x6, row-major
  float u_min[8];
  float u_max[8];
};

// Baseline MPPI sampling/cost/runtime hyperparameters (CLAUDE.md Phase 4.1 Sec.2.2), loaded from
// robot_control.yaml's baseline_mppi.* block via BaselineMppiConfig::loadFromNode() and uploaded
// once at init() -- never re-uploaded per solve() (K/H/dt are fixed for the lifetime of the
// controller; only the small per-tick state/target snapshot changes per solve()).
struct CudaBaselineRuntimeConfig
{
  int num_rollouts;    // K
  int horizon;         // H
  float dt;            // outer-loop step, s (matches outer_decimation_'s 25Hz/20Hz period)
  float lambda;        // MPPI inverse temperature
  float sigma_tau[6];  // exploration std per wrench DOF [Fx,Fy,Fz,Tx,Ty,Tz]
  float q_pos[3];      // position-error cost weights
  float q_quat;        // attitude-error cost weight (scalar, applied to ||e_R||^2)
  float q_vel[6];      // body-twist-error cost weights
  float r_u;           // control-effort cost weight on ||u||^2 (u = 8 allocated thrusters)
  float r_dtau[6];     // control-rate cost weights on (tau_t - tau_{t-1})
  float terminal_mult;  // multiplies q_pos/q_quat/q_vel at the final horizon step only
  float x_wall;        // world X of the wall plane (unilateral contact penalty)
  float wall_k;        // wall penalty stiffness (N/m)
  float wall_b;        // wall penalty damping (N*s/m)
  float preload_fx;    // constant pressing bias (N) added to Fx once p.x <= x_wall
  unsigned long long seed;  // NOLINT(runtime/int) -- must match curand_init()'s own parameter type
};

}  // namespace baseline_mppi
}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__ALGORITHMS__BASELINE_MPPI__BASELINECUDATYPES_H_
