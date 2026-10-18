# `adhere_keyboard_input`

## 职责

人机交互终端 + Xbox 手柄遥操作桥接层（CLAUDE.md Phase 3.2）。通过
`adhere_control_msgs` 定义的 4 话题，以标准 ROS 2 话题（非实时路径）与
`AdhereController` 异步通信，三个可执行文件各自职责正交：

- `fsm_console_node`：主控 FSM 状态切换终端，**仅**发布 `/rov/fsm_command`；
- `state_cmd_console_node`：`DEPTH_HOLD`/`PID_HOLD` 专属数值设定终端，**仅**发布
  `/rov/setpoint_cmd`，由 `fsm_console_node` 按需自动弹出，也可手动单独运行；
- `xbox_teleop_node`：订阅官方 ROS 2 `joy` 包的 `/joy`（`sensor_msgs/msg/Joy`），
  转换为 `/rov/teleop_cmd`（美国手/Mode 2 连续摇杆）与手柄按键边沿触发的
  `/rov/fsm_command`（瞬时状态切换）。

本包**零依赖** MuJoCo/CUDA/`ros2_control`/`adhere_mujoco_hardware`/
`adhere_controller`，可独立部署到一台不安装仿真/控制栈的轻量级远程
笔记本（上位机/GCS）上，通过 ROS 2 DDS 控制运行仿真的 Dell Precision 7920
服务器或未来的 Jetson AGX Xavier 真机。

## 文件

```text
src/
├── fsm_console_node.cpp          # 主控 FSM 状态切换终端 + 动态子终端生命周期管理
├── state_cmd_console_node.cpp    # DEPTH_HOLD/PID_HOLD 专属设定终端（ROS 参数 mode=2|4）
└── xbox_teleop_node.cpp          # /joy -> TeleopCommand/FsmCommand 桥接
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
  [1] state_passive    (推进器全零输出 u = 0 N，自由漂流)
  [2] state_depth_hold (PID 定深控制，自动弹出深度设定终端)
  [3] state_open_loop  (Xbox 开环推力映射，B 伪逆分配)
  [4] state_pid_hold   (6DOF 位置+航向保持，自动弹出设定终端)
  [q] Quit Console     (退出控制台)
------------------------------------------------------------
Current State: [1] state_passive (state_passive)
Select state (1/2/3/4/q):
```

- 订阅 `/rov/fsm_state`（`adhere_control_msgs::msg::FsmState`，**不**再订阅
  `/rov/odom` 等原始硬件话题）在独立的 ROS executor 线程上持续更新菜单中显示
  的当前状态，主线程阻塞在 `std::cin` 上读取用户选择，两者互不阻塞。
- 输入 `1`/`2`/`3`/`4`：发布对应的 `FsmCommand{command=...}` 到
  `/rov/fsm_command`；**不**携带任何数值设定点。
- 输入 `q`：退出控制台（不影响已激活的 FSM 状态）。
- **动态子终端生命周期管理**：只要 `/rov/fsm_state::current_state` 发生变化
  （无论切换来自本控制台的键盘输入还是 Xbox 手柄按键），就先终止上一个已弹出
  的 `state_cmd_console_node` 子终端，再视新状态决定：
  - 新状态是 `DEPTH_HOLD`（`2`）或 `PID_HOLD`（`4`）：以
    `fork()` + `setsid()` + `execl()` 启动一个新的 `gnome-terminal --wait`
    （未安装则回退 `xterm`）窗口运行
    `ros2 run adhere_keyboard_input state_cmd_console_node --ros-args -p mode:=<2|4>`；
    `setsid()` 使子进程自成一个进程组，之后 `killpg()` 才能把该终端窗口与其内部
    的 `ros2 run` 进程一并清理干净（`--wait` 使 gnome-terminal 客户端进程的
    生命周期跟随内部命令，否则裸 `gnome-terminal` 客户端在请求常驻服务端开窗后
    会立刻退出，使这个 pid 变得不可控）；
  - 新状态是 `PASSIVE`（`1`）或 `OPEN_LOOP`（`3`）：只清理旧子终端，不弹出新的。

## `state_cmd_console_node`：`DEPTH_HOLD`/`PID_HOLD` 专属设定终端

ROS 参数 `mode`（`int`，`2` 或 `4`）决定界面；订阅 `/rov/fsm_state` 实时显示
Actual/Target，**仅**发布 `/rov/setpoint_cmd`：

```text
# mode:=2（水面 Z=0.0m，水深输入既可为正也可为负，二者等价：2.50 与 -2.50
# 都代表水面以下 2.50m，内部统一归一化为 target_z = -|input|，钳制在
# [-9.80, -0.15]m）
Current Depth: 2.000 m (Z = -2.000 m)   |   Target Depth: 2.500 m (Z = -2.500 m)
[DEPTH_HOLD] Enter target depth in meters, positive or negative (float, e.g.
2.50 or -2.50 -- both mean 2.50m below the surface), or 'h' to hold current depth:

# mode:=4（`z <val>` 同样接受正负水深输入，归一化方式与 mode:=2 一致）
[PID_HOLD] Enter 'z <val>' (depth), 'y <val>' (lateral), 'x <val>' (wall dist),
'yaw <deg>', 'all <x> <y> <z> <yaw_deg>', or 'h' (lock current pose):
```

`mode:=4` 下的单轴部分更新（`z`/`y`/`x`/`yaw`）会把当前已知的 `target_pose`
（来自 `/rov/fsm_state`）回填进另外 3 个轴，再覆盖被修改的那一个，因此
`z 0.4` 这样的输入**不会**把 `x`/`y`/`yaw` 意外清零。

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
| A | `0` | `PASSIVE`（紧急安全零推力） |
| B | `1` | `PID_HOLD` |
| X | `2` | `DEPTH_HOLD` |
| Y | `3` | `OPEN_LOOP` |

## 独立上位机（GCS）部署

```bash
# 远程笔记本：仅需本包 + adhere_control_msgs + 官方 joy 包，零 MuJoCo/CUDA/ros2_control
ros2 launch adhere_keyboard_input teleop_gcs.launch.py
ros2 launch adhere_keyboard_input teleop_gcs.launch.py dev:=/dev/input/js1
```

启动 `joy_node`（`joy` 包）+ `xbox_teleop_node` + `fsm_console_node` 三者。
与运行仿真/真机的下位机之间**只**通过 ROS 2 DDS 的 4 个 `/rov/*` 话题通信，
不要求两台机器安装相同的软件栈（下位机需要 MuJoCo/CUDA/`ros2_control`，上位机
不需要）。

在单工作站便捷模式下，`adhere_controller/launch/mujoco.launch.py` 的
`launch_teleop`（默认 `true`）参数会自动 `include` 本文件；分布式部署时在
下位机侧传 `launch_teleop:=false`，避免同一个手柄被两份 `joy_node` 争抢。

## 规划中的功能（后续阶段）

| 功能 | 说明 |
|---|---|
| 站位保持/轨迹跟踪目标发送 | 待 `TargetTrajectory.msg` 实现后补充 |
| 标准 12.0s 阵风扰动 benchmark 触发 | 用于复现论文基准实验 |
