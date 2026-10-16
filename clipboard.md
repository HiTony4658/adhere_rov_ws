# Task: Comprehensive Code Audit and Hydrodynamic Enhancement Analysis for `adhere_mujoco_hardware`

Please perform a rigorous, line-by-line audit of all source and header files inside `src/hardwares/adhere_mujoco_hardware/` (including `adhere_mujoco_hardware.cpp/.hpp`, `fossen_hydrodynamics.cpp/.hpp`, `flow_environment_plugin.cpp/.hpp`, `mujoco_viewer.cpp/.hpp`, `adhere_mujoco_node.cpp`, launch files, config files, and tests)[span_1](start_span)[span_1](end_span).

Your objectives are:
1. **Code Audit & Bug Detection**: Check for thread-safety issues, race conditions on `MujocoVisBridge`, memory management, ROS 2 Foxy API compatibility, MuJoCo data/pointer safety, and numerical stability in the 500Hz physics loop[span_2](start_span)[span_2](end_span).
2. **Completeness & Feature Gaps**: Identify any missing architectural features, control hooks, or telemetry requirements.
3. **Hydrodynamic Simulation Accuracy**: Analyze both `kParametricLumped` and `kStonefishGeometryDriven` hydrodynamic modes in `fossen_hydrodynamics.cpp`[span_3](start_span)[span_3](end_span). Propose concrete mathematical or algorithmic enhancements to maximize simulation fidelity (e.g., higher-order fluid-structure interaction, frequency-dependent damping, vortex-induced effects).
4. **Output Format & Interaction Requirement**:
   - Provide your full code analysis, identified issues, and enhancement recommendations in **Simplified Chinese (中文)**.
   - At the end of your response, explicitly **ask the user** (in Chinese) which of the proposed recommendations they would like to modify or implement.
 