# `adhere_keyboard_input`

## 职责

人机交互终端节点。通过 `adhere_control_msgs` 定义的消息，以标准 ROS 2 话题
（非实时路径）与 `AdhereKamrController` 异步通信：

- `fsm_console_node`（已实现，Phase 3.1）：交互式 FSM 终端控制台，发送
  FSM 状态切换/定深目标请求（`FsmCommand.msg` → `/rov/fsm_command`）；
- 未来阶段将补充一键触发标准 12.0s 阵风扰动 profile 等基准实验触发节点。

## 文件

```text
src/fsm_console_node.cpp   # 交互式 FSM 终端控制台
```

## `fsm_console_node`：交互式 FSM 终端控制台

```text
============================================================
  Underwater ROV FSM Control Console (有限状态机终端控制台)
============================================================
  [1] state_passive    (推进器全零输出 u = 0 N，自由漂流)
  [2] state_depth_hold (PID 定深控制，输入目标深度 Z 坐标)
  [q] Quit Console     (退出控制台)
------------------------------------------------------------
Current State: [1] state_passive | Current Z: 0.500 m
Select state (1/2/q):
```

- 订阅 `/rov/odom`（`nav_msgs/msg/Odometry`）在独立的 ROS executor 线程上持续
  更新菜单中显示的"Current Z"，主线程阻塞在 `std::cin` 上读取用户选择，两者
  互不阻塞。
- 输入 `1`：发布 `FsmCommand{command=PASSIVE}`。
- 输入 `2`：提示输入目标深度（浮点数，米），校验后发布
  `FsmCommand{command=DEPTH_HOLD, target_z=<输入值>}`；已处于 `DEPTH_HOLD`
  态时再次输入 `2` 只是更新 `target_z`（不会触发 `AdhereKamrController` 的
  `enter()`，因此不会重置 PID 积分器，见 `adhere_controller_common` README）。
- 输入 `q`：退出控制台（不影响已激活的 FSM 状态）。

### 运行

```bash
ros2 run adhere_keyboard_input fsm_console_node
```

`mujoco_sim.launch.py` 的 `spawn_fsm_console`（默认 `true`）参数会自动在独立的
`gnome-terminal`/`xterm` 窗口中启动本节点；设为 `false` 可改为手动运行上述命令。

## 规划中的功能（后续阶段）

| 功能 | 说明 |
|---|---|
| 站位保持/轨迹跟踪目标发送 | 待 `TargetPose.msg`/`TargetTrajectory.msg` 实现后补充 |
| 标准 12.0s 阵风扰动 benchmark 触发 | 用于复现论文基准实验 |
