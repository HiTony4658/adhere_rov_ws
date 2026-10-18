# Real-time frequency/latency observability (Phase 3.6), published at 10Hz by AdhereController
# on /rov/runtime_diagnostics. Consumed by rt_monitor_node (CLI table), fsm_console_node (1-line
# summary) and the /rov/diagnostics_hud_marker RViz2 TEXT_VIEW_FACING overlay.
std_msgs/Header header

# Measured Trigger Rates (Hz)
float64 hw_loop_hz              # Target: 500.0 Hz
float64 sensor_imu_hz           # Target: 500.0 Hz
float64 sensor_loadcell_hz      # Target: 500.0 Hz
float64 sensor_depth_hz         # Target: 50.0 Hz
float64 sensor_dvl_hz           # Target: 20.0 Hz
float64 estimator_hz            # Target: 500.0 Hz
float64 inner_loop_hz           # Target: 250.0 Hz
float64 outer_loop_hz           # Target: 25.0 Hz
float64 thruster_cmd_hz         # Target: 250.0 Hz (inner update)

# Execution Latencies (ms) & Budgets (ms)
float64 hw_step_exec_ms
float64 estimator_exec_ms
float64 estimator_max_ms
float64 inner_loop_exec_ms
float64 inner_loop_max_ms
float64 inner_loop_budget_ms    # 4.0 ms (for 250 Hz)
float64 outer_loop_exec_ms
float64 outer_loop_max_ms
float64 outer_loop_budget_ms    # 40.0 ms (for 25 Hz)

# Real-time Overrun Counters & Health Flag
uint64 inner_loop_overruns      # Count of ticks where inner_loop_exec_ms > inner_loop_budget_ms
uint64 outer_loop_overruns      # Count of ticks where outer_loop_exec_ms > outer_loop_budget_ms
bool timing_healthy             # True if all frequencies are within 10% of target and latest exec <= budget
