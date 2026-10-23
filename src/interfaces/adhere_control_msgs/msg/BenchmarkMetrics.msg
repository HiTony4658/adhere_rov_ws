# Offline-evaluation/benchmark telemetry (CLAUDE.md Phase 5.0 Stage 7: split out of
# ControllerState, which stays lean FSM/pose feedback for consoles/GCS). Published by
# AdhereController on /rov/benchmark_metrics at the 10Hz diagnostics channel (CLAUDE.md Phase
# 5.0 Stage 3's "diag" MultiRateDecimator channel) -- only benchmark_evaluator_node consumes
# this. current_state/state_name are included so a single sample is self-describing (no need to
# correlate against a separate ControllerState message to know which FSM state produced it).
std_msgs/Header header

int32 current_state                      # Mirrors ControllerState::current_state
string state_name                        # Mirrors ControllerState::state_name

# Phase 3.8 efficiency/chattering/safety telemetry (CLAUDE.md Phase 3.8): computed centrally by
# AdhereController::update() at every 250Hz inner-loop tick from the actually-sent (sanitized +
# saturated) thruster commands, carried forward here at the 10Hz publish rate. cumulative_*
# integrators reset to 0 on every FSM state transition (common/ControllerMetrics.h).
float64 thrust_l2_norm                   # Instantaneous ||u||_2 (N)
float64 thrust_power_proxy               # Instantaneous hydrodynamic power proxy sum_i |u_i|^1.5 (N^1.5)
float64 thrust_slew_rate_l2              # Instantaneous ||(u[k] - u[k-1]) / dt||_2 (N/s) -- chattering indicator
float64 cumulative_energy_l2             # Integrated int ||u||_2^2 dt since state entry (N^2 * s)
float64 cumulative_impulse_l1            # Integrated int ||u||_1 dt since state entry (N * s)
uint8 saturated_thruster_count           # Number of thrusters currently at >= 98% of their per-thruster thruster_max
uint8 active_contact_pad_count           # Number of pads with estimated contact_forces[i] > contact_force_thresh

float64 kern_margin                      # Rhombic kern safety margin M_kern = 1 - K_val
