# `adhere_keyboard_input`

## 职责

人机交互终端 + Xbox 手柄遥操作桥接层（CLAUDE.md Phase 3.2，四元数消息 +
实时诊断于 Phase 3.6 更新，贴壁基线状态切换/遥测/离线评估于 Phase 3.8 更新，
`ADHERE_MPPI_BASELINE` 状态切换/GPU 规划器延迟遥测于 Phase 4.1 更新）。
通过 `adhere_control_msgs` 定义的 5 话题，以标准 ROS 2 话题（非实时路径）
与 `AdhereController` 异步通信，五个可执行文件各自职责正交：

- `fsm_console_node`：主控 FSM 状态切换终端，**仅**发布 `/rov/fsm_command`，
  并订阅 `/rov/runtime_diagnostics` 显示单行计时摘要、订阅
  `/rov/controller_state` 显示单行效率/安全遥测摘要（Phase 3.8）；
- `state_cmd_console_node`：`HOLD_PID_DEPTH`/`HOLD_PID_6DOF`/`ADHERE_PID_WBC`/`ADHERE_SMC_WBC`/
  `ADHERE_MPPI_BASELINE` 专属数值设定终端，**仅**发布 `/rov/setpoint_cmd`，由
  `fsm_console_node` 按需自动弹出，也可手动单独运行；
- `xbox_teleop_node`：订阅官方 ROS 2 `joy` 包的 `/joy`（`sensor_msgs/msg/Joy`），
  转换为 `/rov/teleop_cmd`（美国手/Mode 2 连续摇杆，`geometry_msgs/Vector3
  linear`/`angular`）与手柄按键边沿触发的 `/rov/fsm_command`（瞬时状态切换）；
- `rt_monitor_node`（Phase 3.6 新增，Phase 3.8/4.1 扩展）：订阅
  `/rov/runtime_diagnostics` 与 `/rov/controller_state`，用 ANSI 彩色实时
  刷新一张 9 路频率/延迟表格（末行追加 GPU 规划器求解延迟
  `outer_planner_solve_ms`，Phase 4.1）+ 一张效率/安全表格，超时/饱和/
  偏离目标值标红；
- `benchmark_evaluator_node`（Phase 3.8 新增）：离线评估节点，订阅
  `/rov/controller_state`/`/rov/ground_truth_odom`/`/rov/runtime_diagnostics`，
  在一个滚动窗口（默认 `12.0s`）上聚合出跟踪精度/能耗/平顺性/贴壁安全性的
  Markdown 对比表，供 Phase 4 MPPI 基准对比使用。

本包**零依赖** MuJoCo/CUDA/`ros2_control`/`adhere_mujoco_hardware`/
`adhere_controller`，可独立部署到一台不安装仿真/控制栈的轻量级远程
笔记本（上位机/GCS）上，通过 ROS 2 DDS 控制运行仿真的 Dell Precision 7920
服务器或未来的 Jetson AGX Xavier 真机。

## 文件

```text
src/
├── fsm_console_node.cpp          # 主控 FSM 状态切换终端 + 动态子终端生命周期管理 + 计时/效率摘要
├── state_cmd_console_node.cpp    # HOLD_PID_DEPTH/HOLD_PID_6DOF/ADHERE_PID_WBC/ADHERE_SMC_WBC/ADHERE_MPPI_BASELINE 专属设定终端（ROS 参数 mode=2|4|5|6|7）
├── xbox_teleop_node.cpp          # /joy -> TeleopCommand/FsmCommand 桥接
├── rt_monitor_node.cpp           # /rov/runtime_diagnostics + /rov/controller_state 实时 ANSI 终端表格（Phase 3.6/3.8）
└── benchmark_evaluator_node.cpp  # 滚动窗口跟踪/能耗/平顺性/贴壁安全性 Markdown 对比评估（Phase 3.8）
config/
└── xbox_teleop.yaml              # 摇杆/按键映射参数（死区、曲线、轴/按键索引）
launch/
└── teleop_gcs.launch.py          # 独立上位机启动文件：joy_node + xbox_teleop_node + fsm_console_node
```

## `fsm_console_node`：主控 FSM 状态切换终端

```text
============================================================
  Underwater ROV FSM Control Console (有限状态机终端控制台)
============================================================
  [1] state_idle_passive_none    (推进器全零输出 u = 0 N，自由漂流)
  [2] state_hold_pid_depth (PID 定深控制，自动弹出深度设定终端)
  [3] state_teleop_openloop_6dof  (Xbox 开环推力映射，B 伪逆分配)
  [4] state_hold_pid_6dof   (6DOF 位置+航向保持，自动弹出设定终端)
  [5] state_adhere_pid_wbc (贴壁 PID 基线：两阶段靠壁+法向力 PI 保压)
  [6] state_adhere_smc_wbc (贴壁 SMC 基线：6DOF 滑模控制+法向预压)
  [7] state_adhere_mppi_baseline (纯 CUDA 外环 MPPI，250Hz 零阶保持)
  [q] Quit Console     (退出控制台)
------------------------------------------------------------
Current State: [1] state_idle_passive_none (state_idle_passive_none)
||u||: --N  Power: --  Slew: --N/s  E: --N^2s  Sat: --/8  Pads: --/4  Kern: --
Select state (1/2/3/4/5/6/7/q):
```

- 订阅 `/rov/controller_state`（`adhere_control_msgs::msg::ControllerState`，
  Phase 3.6 替代已删除的 `FsmState`，**不**再订阅 `/rov/ground_truth_odom`
  等原始硬件话题）与 `/rov/runtime_diagnostics` 在独立的 ROS executor 线程上
  持续更新菜单中显示的当前状态与计时摘要，主线程阻塞在 `std::cin` 上读取
  用户选择，两者互不阻塞。
- 输入 `1`/`2`/`3`/`4`/`5`/`6`/`7`：发布对应的 `FsmCommand{command=...}` 到
  `/rov/fsm_command`；**不**携带任何数值设定点。
- 输入 `q`：退出控制台（不影响已激活的 FSM 状态）。
- **动态子终端生命周期管理**：只要 `/rov/controller_state::current_state` 发生变化
  （无论切换来自本控制台的键盘输入还是 Xbox 手柄按键），就先终止上一个已弹出
  的 `state_cmd_console_node` 子终端，再视新状态决定：
  - 新状态是 `HOLD_PID_DEPTH`（`2`）、`HOLD_PID_6DOF`（`4`）、`ADHERE_PID_WBC`（`5`）、
    `ADHERE_SMC_WBC`（`6`）或 `ADHERE_MPPI_BASELINE`（`7`）：以 `fork()` + `setsid()` +
    `execl()` 启动一个新的 `gnome-terminal --wait`（未安装则回退 `xterm`）
    窗口运行
    `ros2 run adhere_keyboard_input state_cmd_console_node --ros-args -p mode:=<2|4|5|6|7>`；
    `setsid()` 使子进程自成一个进程组，之后 `killpg()` 才能把该终端窗口与其内部
    的 `ros2 run` 进程一并清理干净（`--wait` 使 gnome-terminal 客户端进程的
    生命周期跟随内部命令，否则裸 `gnome-terminal` 客户端在请求常驻服务端开窗后
    会立刻退出，使这个 pid 变得不可控）；
  - 新状态是 `IDLE_PASSIVE_NONE`（`1`）、`TELEOP_OPENLOOP_6DOF`（`3`）或其他：只清理旧子终端，
    不弹出新的。

## `state_cmd_console_node`：`HOLD_PID_DEPTH`/`HOLD_PID_6DOF`/`ADHERE_PID_WBC`/`ADHERE_SMC_WBC`/`ADHERE_MPPI_BASELINE` 专属设定终端

ROS 参数 `mode`（`int`，`2`/`4`/`5`/`6`/`7`）决定界面；`4`/`5`/`6`/`7` 四者
共用完全相同的 `z/y/x/roll/pitch/yaw/all/reset/h` 命令语法（`ADHERE_PID_WBC`/
`ADHERE_SMC_WBC` 贴壁后在 FSM 层直接忽略 `x` 分量，`ADHERE_MPPI_BASELINE` 则始终是
自由 6DOF 目标，终端侧无需区分）。订阅 `/rov/controller_state`
实时显示 Actual/Target，**仅**发布 `/rov/setpoint_cmd`（线上协议是
`command_type` + `geometry_msgs/Pose target_pose` 四元数，本终端自带一套
不依赖 Eigen 的 RPY↔四元数小工具，只在界面层保持人类友好的角度输入/显示，
Phase 3.6）：

```text
# mode:=2（水面 Z=0.0m，水深输入既可为正也可为负，二者等价：2.50 与 -2.50
# 都代表水面以下 2.50m，内部统一归一化为 target_z = -|input|，钳制在
# [-9.80, -0.15]m）
Current Depth: 2.000 m (Z = -2.000 m)   |   Target Depth: 2.500 m (Z = -2.500 m)
[HOLD_PID_DEPTH] Enter target depth in meters, positive or negative (float, e.g.
2.50 or -2.50 -- both mean 2.50m below the surface), or 'h' to hold current depth:

# mode:=4（`z <val>` 同样接受正负水深输入，归一化方式与 mode:=2 一致；
# 同时显示四元数 [qw,qx,qy,qz] 与由其换算出的 RPY(deg)）
[HOLD_PID_6DOF] Enter 'z <val>' (depth), 'y <val>' (lateral), 'x <val>' (wall dist),
'roll <deg>', 'pitch <deg>', 'yaw <deg>', 'all <x> <y> <z> <yaw_deg>',
'reset' (roll/pitch -> 0), or 'h' (lock current pose):
```

`mode:=4` 下的单轴部分更新（`z`/`y`/`x`/`roll`/`pitch`/`yaw`）会把当前已知的
`target_pose`（来自 `/rov/controller_state`，四元数换算回 RPY）回填进其余
轴，再覆盖被修改的那一个，因此 `z 0.4` 这样的输入**不会**把 `x`/`y`/`roll`/
`pitch`/`yaw` 意外清零。`reset` 发送 `MODE_LEVEL_ATTITUDE`（服务端用自己的
实时目标重新计算，不依赖本地回填值）。

### `mode:=5|6|7`（`ADHERE_*`）专属的墙面目标实时重定向命令（本阶段新增）

`mode:=5/6/7` 在上述 `z/y/x/roll/pitch/yaw/all/reset/h` 语法之外，额外提供
发布在新话题 `/rov/adhere_target_cmd`（`AdhereTargetCommand.msg`）上的三类
命令（刻意用 `tag`/`target` 前缀，不与上面裸的 `y`/`z`——那两个仍然是贴壁
后沿墙滑动设定点的既有功能——冲突）：

```text
'tag <0..3>'                       # 墙面 QR 码/AprilTag 预设（对应 robot_control.yaml 的
                                    # adhere_wall_tags_y/adhere_wall_tags_depth_m）
'target y <val>'                   # 只改贴壁目标 Y，深度不变
'target d <depth>'                 # 只改贴壁目标深度，Y 不变
'target y <val> d <depth>'         # 同时改 Y 与深度
'target here'                      # 用 ROV 当前 Y/深度作为新目标
```

服务端（`AdhereController`）据此解析出新目标，写入三个贴壁状态共享的
`CtrlInterfaces::adhere_target_`；若新目标与当前目标的 Y 或深度差超过
`adhere_retarget_threshold`（`robot_control.yaml`，默认 `0.06m`），即使已经
贴壁也会强制退回 `STANDOFF_HOMING` 重新定位，而不是带着压墙力横向/纵向
拖动 ROV。

```bash
ros2 run adhere_keyboard_input state_cmd_console_node --ros-args -p mode:=2
```

## `xbox_teleop_node`：美国手（Mode 2）Xbox 手柄遥操作桥

依赖官方 ROS 2 `joy` 包的 `joy_node`（读取 `/dev/input/js0`，发布
`sensor_msgs/msg/Joy` 到 `/joy`）。在 `/joy` 回调中同步发布
`/rov/teleop_cmd`——无需自建定时器，`joy_node` 的 `autorepeat_rate`（默认
`50.0`）已经决定了发布速率。

### 摇杆映射（`config/xbox_teleop.yaml`，默认死区 `0.08` + 三次方灵敏度曲线）

| 摇杆轴 | 默认索引 | 物理动作 | 输出 |
|---|---|---|---|
| 左摇杆上下（`axis_heave`） | `1` | 上(+) | `heave_z > 0`（上浮 +Z） |
| 左摇杆左右（`axis_yaw`） | `0` | 左(+) | `yaw_rate > 0`（逆时针 `+omega_z`） |
| 右摇杆上下（`axis_surge`） | `4` | 上(+) | `surge_x > 0`（朝墙面方向，机体 `-X`） |
| 右摇杆左右（`axis_sway`） | `3` | 左/右 | `sway_y`（沿墙平移） |
| 左扳机（`axis_pitch_trigger`，LT） | `2` | 按下(+) | `pitch_rate > 0`（`+omega_y`）；按住 LB（`button_pitch_reverse`，默认 `4`）反转符号 |
| 右扳机（`axis_roll_trigger`，RT） | `5` | 按下(+) | `roll_rate > 0`（`+omega_x`）；按住 RB（`button_roll_reverse`，默认 `5`）反转符号 |

索引匹配 Linux `xpad` 驱动默认的 8 轴 Xbox 360 风格布局
（`[LX, LY, LT, RX, RY, RT, DX, DY]`）；若实际手柄/驱动索引不同，用
`ros2 topic echo /joy` 核对后在 `config/xbox_teleop.yaml` 中调整。

扳机单侧行程在多数 `xpad` 配置下报告为静止 `+1.0` / 完全按下 `-1.0`（与摇杆
相反的约定），`applyTriggerDeadzone()` 先做 `(1-raw)/2` 归一化到 `[0,1]` 再
应用死区（CLAUDE.md Phase 3.5）。

### 按键映射（瞬时 FSM 切换，上升沿去抖）

| 按键 | 默认索引 | 切换到 |
|---|---|---|
| A | `0` | `IDLE_PASSIVE_NONE`（紧急安全零推力） |
| B | `1` | `HOLD_PID_6DOF` |
| X | `2` | `HOLD_PID_DEPTH` |
| Y | `3` | `TELEOP_OPENLOOP_6DOF` |

## 独立上位机（GCS）部署

```bash
# 远程笔记本：仅需本包 + adhere_control_msgs + 官方 joy 包，零 MuJoCo/CUDA/ros2_control
ros2 launch adhere_keyboard_input teleop_gcs.launch.py
ros2 launch adhere_keyboard_input teleop_gcs.launch.py dev:=/dev/input/js1
```

启动 `joy_node`（`joy` 包）+ `xbox_teleop_node` + `fsm_console_node` 三者。
与运行仿真/真机的下位机之间**只**通过 ROS 2 DDS 的 `/rov/*` 话题通信，
不要求两台机器安装相同的软件栈（下位机需要 MuJoCo/CUDA/`ros2_control`，上位机
不需要）。

在单工作站便捷模式下，`adhere_controller/launch/mujoco.launch.py` 的
`launch_teleop`（默认 `true`）参数会自动 `include` 本文件；分布式部署时在
下位机侧传 `launch_teleop:=false`，避免同一个手柄被两份 `joy_node` 争抢。

## `benchmark_evaluator_node`：离线评估节点（Phase 3.8）

纯 `rclcpp::Node`（不 fork 子进程，风格与 `rt_monitor_node` 一致），订阅
`/rov/controller_state`（主遥测来源）、`/rov/ground_truth_odom`（本包中
**唯一**允许订阅该离线评估专用话题的节点）、`/rov/runtime_diagnostics`。
在一个可配置的滚动窗口（ROS 参数 `window_sec`，默认 `12.0`）上，每 `2s`
打印一张聚合表：

```bash
ros2 run adhere_keyboard_input benchmark_evaluator_node --ros-args -p window_sec:=12.0 -p trigger_flow_gust:=true
```

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

## 规划中的功能（后续阶段）

| 功能 | 说明 |
|---|---|
| 站位保持/轨迹跟踪目标发送 | 待 `TargetTrajectory.msg` 实现后补充 |
