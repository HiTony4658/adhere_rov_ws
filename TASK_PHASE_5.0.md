# Task Phase 5.0: Framework Consolidation — Collapse the Patch Accretion Before `KAMR-MPPI`

> **Before touching any file**, read `CLAUDE.md` in full and the last two sections of `PROGRESS.md`
> (Phase 4.3 + Phase 4.5) following the Mandatory Session Startup Protocol in `CLAUDE.md`
> Section 0. Build with the mandated PATH prefix:
> `export PATH=/usr/bin:/usr/local/cuda/bin:$PATH && source /opt/ros/foxy/setup.bash && colcon build --symlink-install`.

This is a **consolidation milestone**, not a feature milestone. Every change below is a **deletion, a
unification, or a decoupling**. The acceptance bar is explicit: **net lines-of-code and net
concept-count must go DOWN**. Do not add anything not listed here. If a change would raise
complexity, stop and report instead of inventing scaffolding.

---

## Part A — Architectural Diagnosis (verified by reading the tree, 2026-10-22)

The codebase works but has accreted a strong "patch-on-patch" feel since Phase 3.x. The concrete
targets, with evidence:

1. **The three `ADHERE_*` states are ~90% copy-paste.** `src/FSM/StateAdherePid.cpp` (244 LOC),
   `StateAdhereSmc.cpp` (197), `StateBaselineMppi.cpp` (166) are line-for-line identical in: the
   live-retarget bridge, the `STANDOFF_HOMING` convergence gate, the dock gate, the `MODE_*`
   setpoint edge-detection, the teleop stick→pose integrator + tilt clamp, the 3-stage
   `p_d_world_` override, the lever-arm feedforward, and the `B⁺` allocation. The **only** real
   difference is one line: `pid_.computeWrench` vs `smc_.computeWrench` vs ZOH-from-planner.
   `StateBaselineMppi.cpp`'s own comments say its setpoint/teleop block is *"identical to
   StatePidHold.cpp"*. CLAUDE.md calls the sequence "shared," but it is physically pasted into 3
   files — **~600 LOC that should be ~250.**
2. **Two abstract base classes with zero subclasses.** `algorithms/wbc/WbcAllocatorBase.h` and
   `algorithms/mppi/MppiPlannerBase.h` are "data-contract-only" interfaces that nothing
   implements (`BaselineMppiController` deliberately implements neither). Those two directories
   contain *only* these dead headers — premature abstraction posing as architecture.
3. **The FSM state↔int↔string↔object mapping is quadruplicated.** Adding/removing one state means
   editing: `FSMState::checkFsmCommandChange()` (int→enum, `FSM/FSMState.h`),
   `fsmStateNameToInt()` (enum→int, `adhere_controller.cpp:105`), `getNextState()` (enum→ptr,
   `adhere_controller.cpp:958`), the `state_list_` struct + 7 `make_shared` lines
   (`adhere_controller.cpp:397`), `mujoco_viewer.cpp`'s `kFsmStateNames`, and the keyboard nodes.
   Six+ parallel edit sites for one logical fact.
4. **Every `checkChange()` override is noise.** The base `FSMState::checkChange()` already returns
   `checkFsmCommandChange()`; all 7 concrete states re-declare that exact one-liner.
5. **Mirror gain structs + hand-written copy functions.** Each adhere state carries a
   `namespace{}` function copying `ModelParams::Adhere*Gains` field-by-field into
   `Controller::Gains` — two mirror structs per algorithm, synced by hand.
6. **Three bespoke lock-free bridges, three idioms.** `ActiveState`, `OuterLoopCommand`,
   `AdhereTargetState` each reinvent `valid`/`seq`/`changed_pending`; `AdhereTargetState` bolts on
   an awkward `use_current_requested`+`command_seq` deferred-read hack.
7. **Dead-after-subtraction:** `ActiveState::rpy[3]` exists *"only for StateDepthHold's damping"*;
   removing depth-hold strands it.
8. **`adhere_keyboard_input` is 5 unrelated nodes** (FSM console, setpoint console, joystick, RT
   monitor, benchmark evaluator) — operator input, observability, and offline evaluation crammed
   into one "keyboard" package.
9. **RViz environment-marker geometry is hardcoded in C++.** `adhere_mujoco_node.cpp` hardcodes the
   wall (`pos.z=-4.25`, `scale.z=11.5`, `scale.y=20`), water surface, and tank extents, duplicating
   `rov_wall.xml`. This MJCF-vs-C++ geometry mismatch has had to be hand-synced every phase.
10. **`HOLD_PID_DEPTH` (定深) is redundant** with `HOLD_PID_6DOF` and should be removed outright.

---

## Stage 0 — HARD BLOCKER: fix the `adhere_controller::on_configure()` hang (do this first)

`ros2 launch simplified_v1_description mujoco.launch.py` has hung inside
`adhere_controller::on_configure()` since Phase 4.3 (logs `Configuring controller
'adhere_controller'`, then never returns, 70 s+). Consequence: the controller never reaches
`active`, so **no real MuJoCo closed loop has ever run** — Phases 4.3/4.5 were gtest-only by their
own disclosure. Already ruled out: CUDA/GPU init (only touched in `on_activate`), and the Foxy
`switch_controller`-before-`update()`-thread deadlock (`adhere_mujoco_node.cpp` ordering is correct).

1. **Diagnose while hanging** (attach at launch time): `strace -f` across the whole process tree,
   or read `/proc/<pid>/task/*/stack`, or `gdb` if `ptrace_scope` can be relaxed. If no debugger is
   available, bracket every sub-step of `on_configure()` with ordered log lines to localize the
   stall. Prime suspects: a `wait_for_service` / `wait_for_message`, a parameter round-trip through
   the triple-named `adhere_mujoco_node` (the `robot_control.yaml` wildcard-vs-exact-node-name
   gotcha recorded in CLAUDE.md Sec 9), or an executor/mutex interaction.
2. **Fix the root cause** (no timeout band-aid). Document it in `PROGRESS.md` to the standard of
   Phase 4.5's Ctrl+C root-cause writeup.
3. **Gate:** once fixed, `mujoco.launch.py headless:=true use_rviz:=false` must reach `active` and
   run the 500/250/25 Hz loops. **From here on, every stage is verified in real closed loop, not
   gtest alone.** If Stage 0 cannot be fixed within the time box, STOP and report — do not ship a
   third gtest-only phase.

---

## Stage 1 — Subtractions

1. **Delete `HOLD_PID_DEPTH`** (finding 10). Remove `FSM/StateDepthHold.{h,cpp}`; the
   `SixDofPidController` "depth-only mode" branch; `ModelParams` `depth_hold.*` loading and the
   `depth_hold.*` keys in `robot_control.yaml`; the enum/map entries in
   `adhere_controller.{cpp,hpp}`, `common/enumClass.h` (which also still lists only up to
   `ADHERE_MPPI_BASELINE` — clean it up here), all 4 msgs; the console/teleop/monitor references;
   `mujoco_viewer.cpp`'s `kFsmStateNames`; and the depth-hold gtest cases.
2. **Compact `FsmCommand` IDs to `1..7`** (ID-stability rule relaxed, owner-confirmed):
   `1=IDLE_PASSIVE_NONE, 2=TELEOP_OPENLOOP_6DOF, 3=HOLD_PID_6DOF, 4=ADHERE_PID_WBC,
   5=ADHERE_SMC_WBC, 6=ADHERE_MPPI_BASELINE, 7=ADHERE_MPPI_KAMR`, `0=NO_CHANGE`. Update **every**
   `1=…8=…` enumeration (4 msgs, keyboard/monitor nodes, viewer, YAML comments, all READMEs,
   CLAUDE.md Sec 9) and amend **CLAUDE.md Sec 7.5** to record this one-time compaction (IDs remain a
   contract going forward, just no longer frozen to historical numbers).
3. **Remove all auto-spawned setpoint terminals** (finding 8, part). Joystick is the primary
   interface; the controller pose is only an *estimate*, so precise numeric entry is false
   precision — done ad hoc via `ros2 topic pub /rov/setpoint_cmd`. Delete `fsm_console_node`'s
   `spawnStateConsole`/`killChildConsole`/`fork`/`setsid`/`killpg`/`prctl` child-terminal machinery
   and the state-change subscription that drives it; keep only FSM switching + the
   `/rov/controller_state` + `/rov/runtime_diagnostics` status readout. **Delete
   `state_cmd_console_node.cpp` entirely.** Re-evaluate whether Phase 4.5's `OnShutdown`+`pkill`
   orphan cleanup is still needed once no consoles are `gnome-terminal`-spawned; drop it if not,
   and document the decision.
4. **Delete the dead abstractions** (finding 2): `algorithms/wbc/WbcAllocatorBase.h`,
   `algorithms/mppi/MppiPlannerBase.h`, and the now-empty `wbc/`+`mppi/` dirs. The real
   WBC/OSQP + planner interface is reintroduced *with its concrete implementation* when
   `KAMR-MPPI` lands (Phase 4.4+), never speculatively.
5. **Delete redundant `checkChange()` overrides** from all concrete states (finding 4) — the base
   default already does it.
6. **Remove `ActiveState::rpy[3]`** (finding 7): verify no consumer survives depth-hold removal
   (grep); if any state still needs RPY, derive it locally from `quat_w`.

---

## Stage 2 — Unify the FSM layer (the biggest de-bloat)

1. **Single FSM registry** (finding 3). Replace the 4 parallel mappings + the `state_list_` struct
   with one ordered registry of `{id, FSMStateName, string, factory}` entries. Derive
   int→enum (`checkFsmCommandChange`), enum→int (`fsmStateNameToInt`), enum→ptr (`getNextState`),
   and construction all from this one table. Adding a state becomes a one-line registry entry. The
   `.msg` constants remain the authoritative numeric IDs; document that the registry's `string`
   column is the single C++ source for display names (the hardware viewer and keyboard nodes live
   in other packages and cannot include this header — note in each that their name arrays must
   mirror `FsmCommand.msg`).
2. **Extract `PoseSetpointIntegrator`** (findings 1): a ROS-free, header-only, gtest-able helper
   (same style as `common/OutputSafety.h`) owning `p_d_world_`/`q_d_` and the shared `MODE_*`
   edge-detect + teleop stick→pose integration + tilt clamp currently pasted into
   `StatePidHold`/`StateAdherePid`/`StateAdhereSmc`/`StateBaselineMppi`. Parameterize the one real
   difference (whether surge drives X). All four states consume it.
3. **Extract `StateAdhereBase`** (finding 1): carries the entire shared wall sequence — retarget
   bridge, sticky `standoff_homed_`/`docked_` gates, 3-stage `p_d_world_` override, soft-landing
   damp, lever-arm feedforward, `B⁺` allocation, `adhereStageName()`, `targetPose()`. Concrete
   states implement a single virtual hook:
   ```cpp
   virtual Vec6 computeAdhesionWrench(const AdhereStageContext & ctx, double dt) = 0;
   ```
   `StateAdherePid` → `pid_.computeWrench` + force-PI on the docked Fx; `StateAdhereSmc` →
   `smc_.computeWrench`; `StateBaselineMppi` → ZOH from `outer_loop_cmd_` (reports only 2 stages).
   **Target: the three `.cpp`s drop from ~600 to ~250 LOC with identical runtime behavior.**
4. **Collapse the mirror gain structs** (finding 5): have
   `SixDofPidController`/`SixDofSmcController`/`AdherePidController` consume the
   `ModelParams::Adhere*Gains` structs directly (or share one struct definition), deleting the
   per-state `namespace{}` copy functions.
5. **One lock-free bridge primitive** (finding 6): add a small `SeqStampedCommand<T>` /
   `AtomicSnapshot<T>` in `common/` and reimplement `OuterLoopCommand` + `AdhereTargetState` (and
   the `use_current` deferred-read) on top of it, so there is one idiom, not three. Preserve the
   "500 Hz-thread-only read" discipline.

---

## Stage 3 — One rate-division framework

Generalize `common/MultiRateDecimator.h` into a **named-channel registry**: each channel = an
integer decimation against the 500 Hz master clock; `tick()` returns which channels fired this
tick; per-channel `periodSeconds()`; `reset()` re-aligns all channels to tick 0. Route **every**
rate through it and delete the ad-hoc mechanisms in `adhere_controller.cpp`:
`diag_decimation_`/`diag_tick_count_`, the estimator's implicit every-tick cadence (register it
explicitly), the `controller_state` publish cadence, and the depth/DVL sensor-rate diagnostics
(derive from the known 10×/25× decimations instead of `have_last_*_diag_` value-change detection —
keep edge-detection only if still needed to *prove* the hardware actually decimates, with a comment
saying so). Fix the header comment (it name-drops the deleted `StateDepthHold`) and extend
`test/test_multirate_decimator.cpp` for the multi-channel API.

---

## Stage 4 — Sim/Real noise split + environment-marker single source

1. **Sim-injected noise → MJCF** (owner-confirmed; sim-only, hardware is not an `rclcpp::Node`).
   Move the `constexpr`s in `adhere_mujoco_hardware.cpp` (`kGyroNoiseStd`, `kGyroBiasRwStd`,
   `kAccelNoiseStd`, `kAttNoiseStd`, `kDepthNoiseStd`, `kDvlVelNoiseStd`, `kDvlRangeNoiseStd`,
   `kLoadcellNoiseStd`, `kDepthDecimation`, `kDvlDecimation`) into a `SensorNoiseParams` struct with
   `applyMjModelOverrides()` reading `<custom><numeric name="sensor_noise_*">` from `rov_wall.xml` —
   **mirror `HydroParams::applyMjModelOverrides()` exactly** (the existing code comment already names
   this as the correct path for a non-`rclcpp::Node` hardware component). Add the `<custom><numeric>`
   entries to `rov_wall.xml`; keep current values as compiled-in defaults.
2. **Estimator assumed gains → `robot_control.yaml`** (owner-confirmed; measured from the real
   sensors in a real experiment). Move `RovStateEstimator.h`'s hardcoded `static constexpr`
   (`kAttGain`, `kDepthPosGain`, `kDepthVelGain`, `kDvlVelGain`, `kDvlRangeGain`,
   `kContactForceFilterGain`, `kContactVelGain`, `kContactPosGain`) into an `estimator.*` YAML
   block, loaded by `ModelParams::loadFromNode()` and injected via the estimator's constructor.
   Rationale (document in the YAML): the estimator runs inside the `adhere_controller` ROS node,
   which *can* read the YAML — unlike the sim-side std above. Keep current values as defaults so
   `test_sensor_fusion_and_teleop.cpp`'s `<0.015 m/s / <15 mm / <1.0°` RMSE contract passes
   unchanged.
3. **Environment markers from a single source** (finding 9). Stop hardcoding the wall/water/tank
   geometry in `adhere_mujoco_node.cpp`. Derive the RViz `/rov/environment_markers` from the loaded
   `mjModel` geoms (the hardware already owns `model_`; publish the geometry via a small bridge
   struct / topic), or at minimum from one shared parameter block, so `rov_wall.xml` is the single
   source of truth and the markers can never silently desync from the physics again.

---

## Stage 5 — Split `adhere_keyboard_input` + new perception package (finding 8)

Five unrelated nodes → coherent packages. All of these keep **zero** MuJoCo/CUDA/ros2_control deps
so they still build and run standalone on a remote GCS laptop:

- **`commands/adhere_teleop_gcs`** — operator input / GCS: `fsm_console_node` + `xbox_teleop_node`
  + `teleop_gcs.launch.py` + `config/xbox_teleop.yaml`.
- **`commands/adhere_monitor`** — observability / evaluation: `rt_monitor_node` +
  `benchmark_evaluator_node`.
- **`perception/adhere_apriltag_detector`** — a NEW top-level `perception/` category (parallel to
  `hardwares/`), mirroring the real-robot `apriltag_ros` boundary. Houses the Stage-6 detector.
- `state_cmd_console_node` is **deleted** (Stage 1.3), not moved.

Update `CLAUDE.md` Sec 6 package tree + Sec 9 snapshot, every category/package `README.md`, and all
`teleop_gcs.launch.py` / `mujoco.launch.py` include paths.

---

## Stage 6 — AprilTag two-tier adhere target (replaces the popped-up terminal)

1. **Two-tier target.** A coarse preset (YAML `adhere_target_*`, or `/rov/adhere_target_cmd` via
   `ros2 topic pub`) drives `STANDOFF_HOMING` to roughly the right height/Y. A **measured tag
   detection is the final authority** and supersedes the preset: on a fresh detection differing from
   the active target by `> adhere_stage.retarget_threshold`, write it into the Stage-2.5 bridge and
   force a fresh homing pass (reuse Phase 4.5's `changed_pending` path). Document that a received tag
   governs over a later disagreeing preset until explicitly re-presetted.
2. **New `TagDetection.msg`** on **`/rov/tag_detection`**, kept distinct from
   `/rov/adhere_target_cmd` (measurement vs human intent):
   ```
   std_msgs/Header header
   uint8   station_id        # = min tag id of the bundle / 4 (see 6.4); indexes adhere_wall_tags
   float64 target_y          # world Y (m) of the detected bundle centre
   float64 target_depth_m    # depth below surface (m) of the detected bundle centre
   float64 confidence        # [0,1], detection quality / fraction of bundle tags seen
   bool    valid
   ```
3. **`adhere_apriltag_detector` node (Sim-to-Real identical).** It holds the configured wall-tag
   table (reuse `adhere_wall_tags_*`), subscribes to the ROV's **estimated** pose
   (`/rov/controller_state`, never ground truth, to stay Sim-to-Real pure), and when the ROV enters
   a configurable detection range/cone of a station publishes that station's `(Y, depth)` on
   `/rov/tag_detection` with a small configurable Gaussian offset + `confidence` — exercising the
   "preset ≠ actual sticker" case. The **real** detector (`apriltag_ros`, `tag36h11`) publishes the
   identical message on the identical topic; `AdhereController`'s consumer logic is unchanged
   between sim and real.
4. **Physical wall spec — 2×2 AprilTag bundle per station** (document in
   `simplified_v1_description/README.md`, no code). Owner-selected bundle layout:
   - Family `tag36h11`; individual tag black-square side **`s = 80 mm`**; centre-to-centre pitch
     **`p = 160 mm`** both axes (⇒ 80 mm white gap between adjacent tags); bundle black extent
     **240 × 240 mm**; backing plate **300 × 300 mm** with a **matte (anti-glare)** waterproof
     laminate — never glossy (specular glare from pool/underwater point lights kills detection).
   - **IDs: 4 stations × 4 tags = IDs 0–15**, station `k` uses contiguous IDs `4k … 4k+3`, so
     `station_id = min_id / 4`; the bundle's geometric centre = that station's `(Y, depth)`
     centring point:

     | station k | `adhere_wall_tags` (Y, depth) | TL | TR | BL | BR |
     |-----------|-------------------------------|----|----|----|----|
     | 0 | (0.00, 1.00) | 0 | 1 | 2 | 3 |
     | 1 | (0.80, 1.00) | 4 | 5 | 6 | 7 |
     | 2 | (−0.80, 1.50) | 8 | 9 | 10 | 11 |
     | 3 | (1.20, 2.00) | 12 | 13 | 14 | 15 |

   - **`apriltag_ros` `tag_bundles` local layout** (bundle frame at centre, `x`=horizontal/world-Y,
     `y`=vertical/world-up, half-pitch 0.08 m, every tag `size = 0.08`):
     ```
     TL(4k+0): x=-0.08 y=+0.08    TR(4k+1): x=+0.08 y=+0.08
     BL(4k+2): x=-0.08 y=-0.08    BR(4k+3): x=+0.08 y=-0.08
     ```
     The detected bundle-origin pose → the station's `(Y, depth)` target, published as
     `TagDetection`.
   - **Mounting:** all 4 tags coplanar and flush to the wall, rows/cols aligned to world Y
     (horizontal) and Z (vertical, no rotation), bundle centre on the station target point.
   - **Sim note:** the simulated detector does NOT need the 2×2 geometry — it uses the 4 station
     centre points only; the 16 IDs collapse to 4 points in sim. The bundle layout exists solely to
     make the real `apriltag_ros` config ready.

---

## Stage 7 — Message cleanup

1. **Split benchmark telemetry out of `ControllerState`** (owner-confirmed) into
   `BenchmarkMetrics.msg` (`thrust_l2_norm`, `thrust_power_proxy`, `thrust_slew_rate_l2`,
   `cumulative_energy_l2`, `cumulative_impulse_l1`, `saturated_thruster_count`,
   `active_contact_pad_count`, `kern_margin`, plus `current_state`/`state_name` so a sample is
   self-describing) on **`/rov/benchmark_metrics`**, published at the 10 Hz diagnostics channel
   (Stage 3) — only `benchmark_evaluator_node` consumes it. `ControllerState` stays lean FSM/pose
   feedback for the consoles/GCS. Repoint `benchmark_evaluator_node` (now in `adhere_monitor`).
2. **Fix stale comments / dead fields:** `ControllerState::adhere_stage`
   (`DEPTH_ALIGN/WALL_ADHERE` → `STANDOFF_HOMING/SOFT_DOCKING/WALL_ADHERED`); `TeleopCommand`
   consumer list (drop `StateDepthHold`); `SetpointCommand::MODE_DEPTH_ONLY` comment. Audit and
   remove dead YAML keys: `adhere_vz_tol` (self-labeled legacy), `adhere_pid.f_preload` (verify
   unused before removing), and the whole `depth_hold.*` block (dead after Stage 1.1).
3. **Re-document the final topic contract** in CLAUDE.md Sec 3.4 + Sec 9 and the relevant READMEs:
   `/rov/fsm_command`, `/rov/setpoint_cmd`, `/rov/teleop_cmd`, `/rov/controller_state`,
   `/rov/runtime_diagnostics`, `/rov/adhere_target_cmd`, `/rov/tag_detection`,
   `/rov/benchmark_metrics` (+ offline-only `/rov/ground_truth_odom`, `/rov/flow_mode_cmd`).

---

## Stage 8 — Build, closed-loop verify, document, push

1. **Build + unit tests.** `export PATH=/usr/bin:/usr/local/cuda/bin:$PATH && source
   /opt/ros/foxy/setup.bash && colcon build --symlink-install` — zero warnings/errors.
   `colcon test --event-handlers console_direct+` — all gtest pass, zero regressions (the existing
   copyright/cpplint/uncrustify lint failures are the documented repo-wide baseline; introduce no
   *new* lint regressions). **Add gtests** for `PoseSetpointIntegrator`, `StateAdhereBase`, the FSM
   registry, the multi-channel decimator, and the two-tier tag override (preset vs measured
   authority + re-home on detection).
2. **Real MuJoCo closed-loop smoke test (unblocked by Stage 0), mandatory — not gtest-only:**
   - All surviving FSM states switch and run; joystick teleop works with no setpoint terminal.
   - The three `ADHERE_*` run `STANDOFF_HOMING → SOFT_DOCKING → WALL_ADHERED`; the simulated tag
     detector fires within range, publishes `/rov/tag_detection`, and the ROV re-homes onto the
     authoritative tag point, overriding the coarse preset.
   - MJCF sensor-noise overrides and estimator-gain YAML keys both take effect without recompiling;
     RViz environment markers match `rov_wall.xml` with no hand-sync.
   - `/rov/benchmark_metrics` populates `benchmark_evaluator_node`'s table; `/rov/controller_state`
     is lean.
   - Launch + `SIGINT` leaves zero orphan processes (Phase 4.5 regression check), for both
     `teleop_gcs.launch.py` standalone and included from `mujoco.launch.py`.
3. **Consolidation scorecard** (record in `PROGRESS.md`): before/after LOC for the three
   `ADHERE_*` `.cpp`s and the FSM mapping sites; count of deleted files (`StateDepthHold.*`,
   `state_cmd_console_node.cpp`, `WbcAllocatorBase.h`, `MppiPlannerBase.h`), deleted YAML keys, and
   deleted bridge idioms. **Net LOC must be negative.**
4. **Docs + git.** Update all Chinese `README.md` files (functional/architecture only — NO
   changelogs) and log Phase 5.0 in `PROGRESS.md` (chronological, including the Stage-0 root cause
   and the scorecard). Update `CLAUDE.md` Sec 3.4 (topic contract), Sec 6 (package tree), Sec 7.5
   (ID rule), and Sec 9 (snapshot/topics/FSM list). Then `git add -A`, commit with a descriptive
   English message ending in the `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>` trailer,
   and `git push`.

---

## Non-goals (explicitly out of scope for Phase 5.0)

- Implementing `ADHERE_MPPI_KAMR` (ID 7) — it stays a rename-only placeholder; its WBC/OSQP + planner
  interface is reintroduced *with* its concrete code in Phase 4.4+, not speculatively here.
- Adding a simulated camera in MuJoCo — the sim tag detector is a proximity-triggered point
  publisher, not a vision pipeline.
- Any new control law, cost term, or tuning change beyond what a deletion/unification forces
  (behavior must be preserved; this is a refactor, not a retune).
