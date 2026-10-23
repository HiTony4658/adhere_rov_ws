# `adhere_monitor`

## 职责

可观测性/离线评估层（CLAUDE.md Phase 5.0 Stage 5，从原 `adhere_keyboard_input`
拆分而来，与操作员输入/GCS 职责的 `adhere_teleop_gcs` 分离）。两个可执行文件均为
纯 `rclcpp::Node`（不 fork 子进程），只读订阅 `/rov/*` 遥测话题，不发布任何
控制/设定点话题：

- `rt_monitor_node`（Phase 3.6 新增，Phase 3.8/4.1 扩展）：订阅
  `/rov/runtime_diagnostics` 与 `/rov/controller_state`，用 ANSI 彩色实时
  刷新一张频率/延迟表格（末行追加 GPU 规划器求解延迟
  `outer_planner_solve_ms`）+ 一张效率/安全表格，超时/饱和/
  偏离目标值标红；
- `benchmark_evaluator_node`（Phase 3.8 新增）：离线评估节点，订阅
  `/rov/controller_state`/`/rov/ground_truth_odom`/`/rov/runtime_diagnostics`，
  在一个滚动窗口（默认 `12.0s`）上聚合出跟踪精度/能耗/平顺性/贴壁安全性的
  Markdown 对比表，供多基线对比使用。这是本包中**唯一**允许订阅
  `/rov/ground_truth_odom`（离线评估专用话题）的节点。

本包**零依赖** MuJoCo/CUDA/`ros2_control`/`adhere_mujoco_hardware`/
`adhere_controller`，可独立部署到一台不安装仿真/控制栈的轻量级远程
笔记本（上位机/GCS）上。

## 文件

```text
src/
├── rt_monitor_node.cpp           # /rov/runtime_diagnostics + /rov/controller_state 实时 ANSI 终端表格
└── benchmark_evaluator_node.cpp  # 滚动窗口跟踪/能耗/平顺性/贴壁安全性 Markdown 对比评估
```

## `rt_monitor_node`

```bash
ros2 run adhere_monitor rt_monitor_node
```

全屏重绘一张逐通道频率/延迟表（IMU/Loadcell/Depth/DVL/Estimator/Inner/Outer/
GPU 规划器）与一张效率/安全表（推力 L2 范数/功率代理/滑移率/累计能耗/累计
冲量/饱和推进器数/接触垫数/Kern 裕度），任一指标偏离目标值超过 10% 或发生
超时/饱和即标红。

## `benchmark_evaluator_node`

```bash
ros2 run adhere_monitor benchmark_evaluator_node --ros-args -p window_sec:=12.0 -p trigger_flow_gust:=true
```

在一个可配置的滚动窗口（ROS 参数 `window_sec`，默认 `12.0`）上，每 `2s`
打印一张聚合表：

- 跟踪精度（相对地面真值）：Y/Z 方向 RMSE/最大滑移（m）、姿态 RMSE/最大
  误差（deg，由估计姿态与地面真值姿态的相对四元数标量部分算出，与半球
  符号无关）；
- 控制能耗：`E_u`/`I_u`（窗口首尾 `cumulative_energy_l2`/
  `cumulative_impulse_l1` 之差）、平均功率代理；
- 平顺性：推力滑移率 RMS/峰值、饱和占比（`saturated_thruster_count>0` 的
  样本比例）；
- 贴壁安全性：最小 Rhombic Kern 裕度、Kern 违规占比
  （`kern_margin<0`）、4 垫满接触保持率（`active_contact_pad_count==4`）。

`trigger_flow_gust:=true` 时，节点启动时单次（非定时器）发布
`/rov/flow_mode_cmd{data:2}`，触发 12.0s 阵风扰动基准场景。
