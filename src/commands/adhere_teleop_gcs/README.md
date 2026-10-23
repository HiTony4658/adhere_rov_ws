# `adhere_teleop_gcs`

## 职责

操作员输入/地面控制站（GCS）层（CLAUDE.md Phase 5.0 Stage 5，从原
`adhere_keyboard_input` 拆分而来 —— 该包此前把人机交互与可观测性/离线评估两类
不相关的职责混在一起，现按关注点拆成本包与 `adhere_monitor`）。通过
`adhere_control_msgs` 定义的话题，以标准 ROS 2 话题（非实时路径）与
`AdhereController` 异步通信，两个可执行文件职责正交：

- `fsm_console_node`：主控 FSM 状态切换终端，**仅**发布 `/rov/fsm_command`，
  并订阅 `/rov/runtime_diagnostics`/`/rov/controller_state` 显示单行计时/
  效率/安全遥测摘要。Phase 5.0 删除了按状态自动弹出专属设定终端
  （`state_cmd_console_node`）的整套子进程生命周期管理代码——手柄是主要
  交互手段，控制器自身位姿只是估计值，专属终端的"所见即所得"精度是假精度；
  手动数值设定点（深度/位姿/贴壁目标）改为直接
  `ros2 topic pub /rov/setpoint_cmd`/`/rov/adhere_target_cmd`；
- `xbox_teleop_node`：订阅官方 ROS 2 `joy` 包的 `/joy`（`sensor_msgs/msg/Joy`），
  转换为 `/rov/teleop_cmd`（美国手/Mode 2 连续摇杆，`geometry_msgs/Vector3
  linear`/`angular`）与手柄按键边沿触发的 `/rov/fsm_command`（瞬时状态切换）。

本包**零依赖** MuJoCo/CUDA/`ros2_control`/`adhere_mujoco_hardware`/
`adhere_controller`，可独立部署到一台不安装仿真/控制栈的轻量级远程
笔记本（上位机/GCS）上，通过 ROS 2 DDS 控制运行仿真的 Dell Precision 7920
服务器或未来的 Jetson AGX Xavier 真机。

## 文件

```text
src/
├── fsm_console_node.cpp   # 主控 FSM 状态切换终端 + 计时/效率摘要（Phase 5.0 简化，无子终端）
└── xbox_teleop_node.cpp   # /joy -> TeleopCommand/FsmCommand 桥接
config/
└── xbox_teleop.yaml       # 摇杆/按键映射参数（死区、曲线、轴/按键索引）
launch/
└── teleop_gcs.launch.py   # 独立上位机启动文件：joy_node + xbox_teleop_node + fsm_console_node
```

## `fsm_console_node`：主控 FSM 状态切换终端

```text
============================================================
  Underwater ROV FSM Control Console (有限状态机终端控制台)
============================================================
  [1] state_idle_passive_none      (推进器全零输出 u = 0 N，自由漂流)
  [2] state_teleop_openloop_6dof   (Xbox 开环推力映射，B 伪逆分配)
  [3] state_hold_pid_6dof          (6DOF 位置+航向保持，手动设定点见下)
  [4] state_adhere_pid_wbc         (贴壁 PID 基线：三阶段靠壁+法向力 PI 保压)
  [5] state_adhere_smc_wbc         (贴壁 SMC 基线：6DOF 滑模控制+法向预压)
  [6] state_adhere_mppi_baseline   (纯 CUDA 外环 MPPI，250Hz 零阶保持)
  [q] Quit Console     (退出控制台)
------------------------------------------------------------
手动数值设定点（深度/位姿/贴壁目标）请用 ros2 topic pub 直接发布到
/rov/setpoint_cmd 或 /rov/adhere_target_cmd（不再自动弹出专属终端）。
------------------------------------------------------------
Current State: [1] state_idle_passive_none (state_idle_passive_none)
Timing: Inner --Hz  Outer --Hz  Status: --
||u||: --N  Power: --  Slew: --N/s  E: --N^2s  Sat: --/8  Pads: --/4  Kern: --
Select state (1/2/3/4/5/6/q):
```

- 订阅 `/rov/controller_state`（`adhere_control_msgs::msg::ControllerState`）
  与 `/rov/runtime_diagnostics` 在独立的 ROS executor 线程上持续更新菜单中
  显示的当前状态与计时/效率摘要，主线程阻塞在 `std::cin` 上读取用户选择，
  两者互不阻塞。
- 输入 `1`-`6`：发布对应的 `FsmCommand{command=...}` 到 `/rov/fsm_command`；
  **不**携带任何数值设定点（`7=ADHERE_MPPI_KAMR` 仍是未实现的重命名占位，
  本终端尚未提供其菜单项）。
- 输入 `q`：退出控制台（不影响已激活的 FSM 状态）。

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
| Y | `3` | `TELEOP_OPENLOOP_6DOF` |

（Phase 5.0 删除了 `HOLD_PID_DEPTH` 状态，X 键原有的对应绑定随之删除。）

## 独立上位机（GCS）部署

```bash
# 远程笔记本：仅需本包 + adhere_control_msgs + 官方 joy 包，零 MuJoCo/CUDA/ros2_control
ros2 launch adhere_teleop_gcs teleop_gcs.launch.py
ros2 launch adhere_teleop_gcs teleop_gcs.launch.py dev:=/dev/input/js1
```

启动 `joy_node`（`joy` 包）+ `xbox_teleop_node` + `fsm_console_node` 三者。
与运行仿真/真机的下位机之间**只**通过 ROS 2 DDS 的 `/rov/*` 话题通信，
不要求两台机器安装相同的软件栈（下位机需要 MuJoCo/CUDA/`ros2_control`，上位机
不需要）。

在单工作站便捷模式下，`simplified_v1_description/launch/mujoco.launch.py` 的
`launch_teleop`（默认 `true`）参数会自动 `include` 本文件；分布式部署时在
下位机侧传 `launch_teleop:=false`，避免同一个手柄被两份 `joy_node` 争抢。
