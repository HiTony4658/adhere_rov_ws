# adhere_rov_ws 开发进度记录

本文件集中记录工作空间的阶段性开发历史、已修改/新增文件清单与量化验证结果。
各包 `README.md` 仅保留纯功能性说明与运行教程，不包含逐步修改记录。

---

## Phase 1（2026-10-15 之前，commit `d9fd900`）

搭建 5 大类、6 个 ROS 2 Foxy 软件包（`adhere_control_msgs`、`simplified_v1_description`
（当时名为 `adhere_rov_description`）、`adhere_mujoco_hardware`、`adhere_controller_common`、
`adhere_kamr_controller`、`adhere_keyboard_input`）的目录骨架、`package.xml`/
`CMakeLists.txt`（源码构建目标以 `# TODO(Phase N)` 注释占位）与全量中文 `README.md`。
`colcon build --symlink-install` 在空骨架下可成功通过（6/6）。无任何 `.hpp`/`.cpp` 实现。

---

## Phase 2（2026-10-15）：`simplified_v1_description` + `adhere_mujoco_hardware` 实现

### 目标

在零控制输入（`u = 0`）下，验证 `AdhereMujocoHardware` 500Hz 物理闭环（MuJoCo 物理
步进 + C++ 移植的 Fossen 水动力学 + 可插拔水流环境）能否驱动 ROV 在 MuJoCo 内嵌
窗口与 RViz2 中同步呈现正确的被动随波漂流（"随波逐流"）行为。

### 包 1：`src/descriptions/simplified_v1_description`

- 将 `adhere_rov_description` 重命名为 `simplified_v1_description`（含 `package.xml`
  的 `<name>`、`CMakeLists.txt` 的 `project()`、工作空间内所有交叉引用）。
- 从只读参考仓库 `~/Research/mujoco_mppi_ws/assets/rov_wall.xml` 原样复制至
  `mjcf/rov_wall.xml`。
- 新增 `xacro/robot.xacro`（机体 `0.2x0.6x0.4m` box、4 个接触垫球体、8 个推力器
  标记、壁面几何，坐标与 MJCF 逐项核对一致）、`xacro/ros2_control.xacro`
  （`<ros2_control>` 标签：8 个 `effort` 命令接口 + 36 个状态接口，命名与
  `CLAUDE.md`/各 README 文档一致）、`urdf/simplified_v1.urdf.xacro`（顶层入口）。
- 新增 `config/visualize.rviz` 与 `launch/visualize.launch.py`（`robot_state_publisher`
  + `rviz2`，无 `joint_state_publisher`/GUI 依赖——模型无可动 URDF 关节）。

### 包 2：`src/hardwares/adhere_mujoco_hardware`

- `fossen_hydrodynamics.hpp/.cpp`：`~/Research/mujoco_mppi_ws/src/env/hydrodynamics.py`
  的逐条 C++ 移植（`HydroParams`/`GustEvent`/`FossenHydrodynamics::compute`，附加
  质量重力偏置、Morison 耦合二次阻力、Venturi 近壁吸附、自由液面渐变、CoB 回正
  力矩）。
- `flow_environment_plugin.hpp/.cpp`：`FlowEnvironmentPlugin` 策略接口 +
  `CalmWaterPlugin`（默认）+ `StationKeepingGustPlugin`（`0.3 m/s` 定常侧向流 +
  `t∈[3.0,4.2]s`/`[7.0,8.2]s` 两段半正弦阵风，峰值 `0.9/0.2 m/s`）+
  `createFlowEnvironment()` 工厂函数。数值常量与 Python 参考逐位一致。
- `mujoco_vis_bridge.hpp`：500Hz↔60Hz 快照桥（POD + `std::mutex`，`try_lock()` 写入）。
- `mujoco_viewer.hpp/.cpp`：独立线程 GLFW/OpenGL 60Hz 渲染（`headless` 可关闭），
  渲染接触垫受力柱状图与环境流速箭头。
- `adhere_mujoco_hardware.hpp/.cpp`：`AdhereMujocoHardware`（Foxy
  `BaseInterface<SystemInterface>`，`configure/export_state_interfaces/
  export_command_interfaces/start/stop/read()/write()`，`read()`/`write()` 均无
  `time/period` 参数）。水动力通过仅本进程内安装的 `mjcb_passive` 回调注入
  `qfrc_passive`，与 Python 参考的调用时机、目标累加数组完全一致，保证数值一致性。
- `adhere_mujoco_node.cpp`：单进程宿主，构造 `hardware_interface::ResourceManager`
  + `controller_manager::ControllerManager`（专属线程 500Hz `read/update/write`
  循环，本阶段无控制器挂载），附加 100Hz 定时器发布 TF（`world->base_link`）、
  `nav_msgs/Odometry`（`/rov/odom`）、`sensor_msgs/JointState`（空，模型无可动关节）、
  `visualization_msgs/MarkerArray`（`/rov/environment_markers`：壁面 + 4 个接触垫
  受力柱状图）。
- `CMakeLists.txt`：手动探测 MuJoCo（`$MUJOCO_DIR` → `~/.mujoco/mujoco-*` →
  `/opt/mujoco` → `/usr/local`，本机命中 `~/.mujoco/mujoco-3.16.0`）、
  `pkg_check_modules(glfw3)`、`find_package(OpenGL)`、`find_package(Eigen3)`；
  构建插件库 + `adhere_mujoco_node` 可执行文件；`pluginlib_export_plugin_
  description_file` 对接 `adhere_mujoco_hardware.xml`（已取消注释）。
- `config/hydro_wave_params.yaml`：`controller_manager`（`update_rate: 500`）实际
  运行参数 + 两套水流预设数值的人工参考记录（非运行时解析）。
- `launch/mujoco_sim.launch.py` + `config/mujoco_sim.rviz`：`wave_profile`/
  `headless`/`use_rviz` 三个启动参数。

### 包间依赖说明

`adhere_mujoco_hardware` 在运行时读取 `simplified_v1_description` 的 `mjcf/` 共享
目录，`simplified_v1_description` 的 `ros2_control.xacro` 又以插件名引用
`adhere_mujoco_hardware`——这是一个合法的双向**运行时**耦合，但若在两个
`package.xml` 中都声明为依赖会导致 colcon 构建顺序出现环（无法拓扑排序）。
两侧均保留代码注释说明该耦合，但不在 `package.xml` 中声明。

### 数值校验结果

用 3 组随机 `(com_pos_world, orientation, ang_vel_world, lin_vel_world, t)` 状态，
分别调用 Python 参考实现（`station_keeping_hydro_params()` 预设）与本仓库 C++
`FossenHydrodynamics::compute()`，10 位小数精度下逐位一致（远小于 `1e-5` 要求）：

| 状态 | F_world (N) | T_world (N·m) |
|---|---|---|
| case 0 | `[-54.6364066, 68.9020031, 247.3705406]` | `[-0.4375000, 0.1660000, -0.3180000]` |
| case 1 | `[59.6931951, 40.0659866, 232.8637115]` | `[-0.9663371, -4.5848097, -0.4907295]` |
| case 2 | `[-1.0844126, 10.5000000, 257.0220000]` | `[0.0000000, 0.0000000, 0.0000000]` |

### 闭环仿真验证结果（`u = 0`）

- **`visualize.launch.py`**：RViz2 正确渲染机体 box、4 个接触垫、8 个推力器标记，
  几何相对位置与 `rov_wall.xml` 一致（已截图人工比对）。
- **`wave_profile:=calm`**：ROV 初始姿态与壁面轻微重叠（`pad` 中心距壁面 `X=0`
  仅 `~0.01m`，小于垫半径 `0.035m`），`u=0` 下接触反力将其推离壁面；随后因净
  正浮力（约 `+1.96 N`，`= rho*V*g + (m_aug-m)*g - m_aug*g`）缓慢上浮至自由液面
  （`z_surface=1.0m`）附近并稳定（约 10~15s 内 `z: 0.5 -> 1.003m`），全程 `Y`
  方向位移 `< 1e-17 m`（数值零）。MuJoCo 窗口可见 ROV 漂浮于水面线附近。
- **`wave_profile:=station_keeping_gust`**：机体 `Y` 方向速度在约 `8~12s` 内单调
  收敛至定常流速 `0.300000 m/s`（`X`/`Z` 方向速度收敛至 `~0`），与 `0.3 m/s`
  定常侧向流的理论终值完全一致；阵风窗口（`t∈[3.0,4.2]s`、`[7.0,8.2]s`）内可观察
  到额外的瞬态扰动。
- **RViz2 与 MuJoCo 双可视化同步**：`/rov/odom` 稳定以 100Hz 发布，`TF`/
  `MarkerArray` 与内嵌 MuJoCo 窗口中的机体姿态/受力柱状图实时一致。

### 已知近似 / 后续阶段待办

- `imu_sensor` 的线加速度通过世界系速度有限差分近似给出（`rov_wall.xml` 无专用
  加速度计传感器），已在代码与 README 中注明。
- `FossenHydrodynamics`/`FlowEnvironmentPlugin` 当前实现于 `adhere_mujoco_hardware`
  内部（本阶段尚无控制器消费）；待 `adhere_kamr_controller` 实现时，按
  `CLAUDE.md` §6 的共享架构意图，应将 `FossenHydrodynamics` 迁移至
  `adhere_controller_common`，避免未来重复实现。
- `config/hydro_wave_params.yaml` 中的水流预设数值目前仅作人工参考，未在运行时
  解析；实际生效的常量位于 `flow_environment_plugin.cpp`。

### 构建与运行环境备注

- 本机 MuJoCo 3.16.0 安装于 `~/.mujoco/mujoco-3.16.0/`（非系统 `pkg-config` 可见）。
- 本机默认 `python3` 指向 miniconda（缺少 `pkg_resources`），需在 `colcon build`
  前将 `/usr/bin` 置于 `PATH` 前部（或 `conda deactivate`）方可正常构建。
- `ros-foxy-joint-state-publisher-gui`/`joint_state_publisher` 均未安装；因模型
  无可动 URDF 关节，`visualize.launch.py` 不依赖它们。

---

## Phase 2.5（2026-10-15）：URDF 壁面解耦、`.gitkeep` 清理、运行时造浪机切换

### 目标

修正 Phase 2 遗留的三个工程/易用性问题：机器人 URDF 污染了环境几何（壁面）、
已有实际文件的目录仍残留 `.gitkeep`、水流环境只能在启动时静态绑定。

### 包 1：`src/descriptions/simplified_v1_description`

- `xacro/robot.xacro`：删除 `<link name="world"/>`、`<joint name="world_to_base">`、
  `<link name="wall">`、`<joint name="world_to_wall">`。`base_link` 成为整个 URDF
  树的唯一根连杆（`check_urdf` 校验：`root Link: base_link has 12 child(ren)`）。
  壁面几何只保留在 `mjcf/rov_wall.xml`（MuJoCo 物理）与 `adhere_mujoco_node` 发布的
  `/rov/environment_markers`（RViz2 可视化）两处。
- `urdf/simplified_v1.urdf.xacro`、`xacro/ros2_control.xacro`：删除不再需要的
  `wave_profile` xacro 参数/硬件 param（环境切换改为运行时话题，见下）。
- `config/visualize.rviz`：`Fixed Frame` 由 `world` 改为 `base_link`（URDF 不再有
  `world` 连杆）。

### 包 2：`src/hardwares/adhere_mujoco_hardware`

- `AdhereMujocoHardware`：`configure()` 不再按 `wave_profile` 硬件参数在启动时
  选择环境插件，而是始终同时构造 `CalmWaterPlugin`/`StationKeepingGustPlugin`
  两个实例，并在 `start()`/`configure()` 中把运行时模式恒定初始化为 `1`
  （静水）。新增 `std::atomic<int> wave_mode_`、`std::atomic<double>
  wave_trigger_time_`、`std::atomic<double> sim_time_atomic_` 三个原子状态，
  以及线程安全的 `setWaveMode(int)` / `waveMode()` / 静态 `activeInstance()`
  （复用既有的 `mjcb_passive` 单实例回调指针模式）。
- 相对造浪时钟：`applyHydrodynamics()`/`write()` 在模式为 `2` 时，一律用
  `τ = max(0, d->time - wave_trigger_time_)` 而非原始仿真时间查询
  `StationKeepingGustPlugin`，确保在仿真运行到任意时刻切换造浪都能从阵风
  `τ=0` 重新播放；`setWaveMode(2)` 每次调用都会重新锁存 `t_trigger`。
- 新增 `wave_console_node.cpp`：交互式终端控制台，发布 `/rov/wave_mode_cmd`
  （`std_msgs/msg/Int32`，`1`=静水/`2`=造浪），本地维护当前模式用于显示提示符，
  `q` 退出。
- `adhere_mujoco_node.cpp`：新增 `/rov/wave_mode_cmd` 订阅者，通过
  `AdhereMujocoHardware::activeInstance()` 转发给当前存活的硬件实例。
- `CMakeLists.txt`/`package.xml`：新增 `std_msgs` 依赖、`wave_console_node`
  可执行目标。
- `launch/mujoco_sim.launch.py`：删除 `wave_profile` 启动参数；新增
  `spawn_wave_console`（默认 `true`）参数，自动检测 `gnome-terminal`/`xterm`
  并在独立终端窗口中启动 `wave_console_node`（均不可用时静默跳过，用户可手动
  `ros2 run adhere_mujoco_hardware wave_console_node`）。

### 仓库清理

- 删除 6 个已有真实文件、残留 `.gitkeep` 占位符的目录下的 `.gitkeep`
  （`simplified_v1_description/{urdf,config,xacro,launch}`、
  `adhere_mujoco_hardware/{src,include/adhere_mujoco_hardware}`）；其余仍为空的
  骨架目录（`adhere_controller_common`/`adhere_kamr_controller`/
  `adhere_keyboard_input`/`adhere_control_msgs` 的子目录）保留 `.gitkeep`。

### 闭环验证结果（`headless:=true use_rviz:=false spawn_wave_console:=false`）

- `xacro src/descriptions/simplified_v1_description/urdf/simplified_v1.urdf.xacro | check_urdf`：
  `root Link: base_link has 12 child(ren)`（单根连杆契约通过）。
- `colcon build --symlink-install`（全工作空间 6/6 包）与
  `colcon build --packages-up-to simplified_v1_description adhere_mujoco_hardware`
  均零警告通过。
- 启动日志确认默认模式：`AdhereMujocoHardware: starting in Calm Water mode`。
- 向 `/rov/wave_mode_cmd` 发布 `{data: 2}` 后，日志立即打印
  `Wave mode switched: calm -> gust`；`/rov/odom` 的 `twist.twist.linear.y`
  从切换前的 `~0`（静水下仅 Z 方向净浮力漂移）在触发后数秒内明显抬升至
  `0.4~0.5 m/s` 量级（随后随阵风窗口波动），确认相对造浪时钟从切换时刻`τ=0`
  正确重放两段阵风，而非被已运行的仿真时长"吃掉"触发窗口。再次发布
  `{data: 1}` 后日志打印 `Wave mode switched: gust -> calm`，环境流速立即归零。

---

## Phase 2.6（2026-10-15）：预控制器校验、纯 Xacro 解耦、完整交互式 MuJoCo GUI、500Hz 物理管线修复

### 目标

在进入 Phase 3（实现控制器包）之前，修复代码审计中发现的 4 个用户可见问题与
5 个隐藏工程 bug，确保 `500Hz` 仿真基础零延迟、零耦合。

### 四个用户可见问题的修复

1. **Xacro 不再携带 `headless`**：`simplified_v1.urdf.xacro`/`ros2_control.xacro`
   删除 `<xacro:arg name="headless"/>` 与 `<param name="headless">`；`headless`
   改为 `adhere_mujoco_node` 的标准 ROS 2 节点参数，经新增的
   `AdhereMujocoHardware::setHeadless(bool)` 在 `configure()` 之后、
   `start_components()`/`start()` 之前到达硬件插件（`MujocoViewer` 的构造也
   随之从 `configure()` 推迟到 `start()`）。
2. **删除 `config/hydro_wave_params.yaml`**（`git rm`）：其两套水流预设数值表
   迁移为 `adhere_mujoco_hardware/README.md` 中的 Markdown 表格。
3. **RViz TF 坐标系缩放**：`visualize.rviz`/`mujoco_sim.rviz` 的 `TF` 显示新增
   `Marker Scale: 0.12`，与推力器/接触垫坐标系尺度一致，不再遮挡机体。
4. **`MujocoViewer` 升级为完整交互式窗口**：仿照官方 `simulate` 应用，新增
   鼠标相机环绕/平移/缩放、双击选中物体 + `Ctrl`+拖拽施加外部力/力矩
   （`mjvPerturb`）、左侧边栏（仿真运行/暂停/复位、造浪模式切换、渲染选项）、
   右侧边栏（8 个推力器手动滑条 + 实时遥测浮层）。详见该包 README 的
   "`MujocoViewer`：仿照官方 `simulate` 的完整交互式窗口"一节。

### 五个隐藏工程 bug 的修复

5. **零延迟两段式 `mj_step`**：`start()` 改为 `mj_resetData` + `mj_step1`；
   `write()` 改为 `mj_step2` + `mj_step1`（而非单次整体式 `mj_step`），使
   `read()` 读到的 `xipos`/`xquat`/`cvel`/`sensordata` 永远是零延迟的"当前拍"
   数据，而非落后一拍。
6. **接触垫坐标不再硬编码**：`configure()` 新增对 `pad_TL_site..pad_BR_site`
   的 `model_->site_pos` 查询，缓存进新增成员 `pad_yz_`，`read()` 的
   `v_sep,i` 计算改用该成员而非本地 `constexpr` 数组。
7. **`robot_state_publisher` + 环境流速箭头 Marker**：`mujoco_sim.launch.py`
   新增 `robot_state_publisher` 节点（发布 `/robot_description` 与完整静态
   TF 树）；`adhere_mujoco_node.cpp` 的 `/rov/environment_markers` 新增
   `ns="flow_velocity"` 的世界系蓝色箭头，与 `MujocoViewer` 自绘箭头同源。
8. **消除 `claim_state_interface` 跨线程数据竞争**：`adhere_mujoco_node.cpp`
   不再用 `resource_manager->claim_state_interface()` 跨线程抢占状态接口，
   改为在 `100Hz` 发布定时器中读取 `AdhereMujocoHardware::visBridge()` 的
   线程安全快照（`qpos`/`qvel`/`pad_force`/`flow_vel_world`），自行换算世界系
   位姿、机体系速度/角速度。
9. **严格 `500Hz` 时钟 + GUI 负载保护**：`physics_thread` 由 `rclcpp::Rate`
   改为 `std::chrono::steady_clock::sleep_until` 的严格 `2.0 ms` 周期循环，
   并对每拍耗时做指数滑动平均（`actual_hz_ema`）写入
   `MujocoVisBridge::actual_physics_hz`；`MujocoViewer::renderLoop()`
   每帧仅用一次 `try_lock()`/`lock_guard` 读取快照，不重复加锁。

### `MujocoVisBridge` 扩展

新增双向控制通道（均为无锁 `std::atomic`）：GUI -> 硬件的 `paused`、
`reset_request`、`wave_mode_cmd`（`0`=无请求，`1`/`2`=切换请求，边沿触发避免
重复锁存造浪时钟）、`manual_ctrl[8]`（叠加式手动推力器指令）、
`xfrc_perturb[6]`（交互扰动力/力矩）；硬件 -> GUI 的 `actual_physics_hz`、
`sim_time`、`wave_mode_active`、`wave_tau`。批量快照（`qpos`/`qvel`/`ctrl`/
`pad_force`/`pad_v_sep`/`flow_vel_world`）仍沿用 `std::mutex` + `try_lock()`
策略。

### 闭环验证结果（`headless:=true use_rviz:=false spawn_wave_console:=false`）

- `xacro .../simplified_v1.urdf.xacro`：零 `headless` 残留；`check_urdf` 通过，
  `base_link` 仍为单根连杆。
- `colcon build --symlink-install`（全工作空间 6/6 包）与
  `colcon build --packages-up-to simplified_v1_description adhere_mujoco_hardware`
  均零警告、零错误通过。
- `ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py headless:=true
  use_rviz:=false spawn_wave_console:=false`：`robot_state_publisher` 与
  `adhere_mujoco_node` 均正常启动，无 FATAL/异常。
- 临时插入的诊断日志确认：`100Hz` 遥测定时器稳定触发且 `TelemetrySnapshot`
  有效（`valid=1`），确认 `visBridge()` 快照路径（替代
  `claim_state_interface`）工作正常；`physics_thread` 的
  `actual_physics_hz` 实测 `499.96 Hz` / `500.03 Hz`（两次采样，相距 2s），
  证明 `steady_clock::sleep_until` 严格 `500Hz` 时钟在 RViz2/GUI 负载下不漂移。
  诊断日志验证后已从代码中移除。
- 本机 DDS（CycloneDDS）在本次会话的沙箱化多次独立 shell 调用之间无法稳定
  交付数据样本（`ros2 topic echo`/`rclpy` 订阅者在能通过 `ros2 topic info`
  看到已匹配的 publisher 的情况下仍收到 0 条消息），判定为本次会话沙箱环境
  的进程间网络发现限制，与本次代码改动无关（已用进程内诊断日志绕过验证）。

---

## Phase 2.7（2026-10-15）：移除终端造浪控制台、mjUI 面板尺寸/点击修复

### 目标

Phase 2.6 发布后，在真实桌面会话中实际操作 MuJoCo 交互式窗口时发现两个新问题，
外加一项简化需求：

1. **终端造浪控制台冗余**：`MujocoViewer` 左侧边栏已提供等效的 `1`/`2` 键与
   "Wave Environment" 单选按钮，`wave_console_node`（独立终端 + 轮询输入）不再
   必要。
2. **左右侧边栏溢出窗口**：`mjr_makeContext(model_, &con_, mjFONTSCALE_150)`
   在非 HiDPI 显示器上误用 150% 字体主题缩放——`mjui_resize()`/`mjui_render()`
   按该缩放后的主题间距计算面板的真实渲染宽度，与 `ui0_.width`/`ui1_.width`
   指定的 `310px` 不一致，导致两侧面板实际渲染宽度远超 `310px` 并溢出窗口
   右边缘。
3. **点击侧边栏内控件无响应**：`MujocoViewer::onCursorPos`/`onMouseButton` 把
   GLFW 原生的顶部起点（top-down）`y` 坐标直接写入 `uistate_.y`，而
   `mjrRect`/`mjuiState` 系列 API 全部遵循 MuJoCo/OpenGL 的底部起点
   （bottom-up）约定（`mjrRect` 的字段字面上就叫 `bottom` 而非 `top`）。
   未做翻转时，`mjui_event()` 内部按 `y` 坐标做的"命中测试"（判断点击落在
   哪一个条目上）系统性地算错，导致点击在侧边栏内几乎总是找不到对应控件、
   看起来"没有响应"。

### 修复内容

1. **删除 `wave_console_node`**：`git rm src/.../src/wave_console_node.cpp`；
   `CMakeLists.txt` 删除其 `add_executable`/`ament_target_dependencies`/
   `install(TARGETS ...)` 条目；`launch/mujoco_sim.launch.py` 删除
   `spawn_wave_console` 参数与自动弹出终端的 `OpaqueFunction`
   （`gnome-terminal`/`xterm` 探测逻辑整体移除）。`/rov/wave_mode_cmd` 话题
   订阅本身保留在 `adhere_mujoco_node.cpp` 中，作为脚本化/远程控制的入口
   （与 `MujocoViewer` 的 `1`/`2` 键是两个等价入口，都调用同一个
   `AdhereMujocoHardware::setWaveMode(int)`）。
2. **`mjFONTSCALE_150` -> `mjFONTSCALE_100`**（`mujoco_viewer.cpp`）：本机显示器
   framebuffer 像素与窗口像素 1:1（非 HiDPI），没有使用 150% 缩放的依据。
3. **GLFW top-down -> MuJoCo bottom-up 的 `y` 坐标翻转**（`mujoco_viewer.cpp`
   `onCursorPos`/`onMouseButton`）：新增 `last_raw_x_`/`last_raw_y_` 成员，
   单独保存"原始"（未翻转）光标位置供相机环绕/平移/缩放与 `mjvPerturb`
   拖拽增量计算使用（这部分不受此 bug 影响，继续用原始 delta）；同时计算
   `y_mj = framebuffer_height - y` 写入 `uistate_.y`，专供 `mjui_event()` 使用。

### 面板宽度行为说明（非 bug，设计既如此）

mjUI 面板宽度是**固定像素值**（`kSidePanelWidth = 310`），不是窗口宽度的百分比；
这是 MuJoCo 官方 `simulate` 应用本身的惯例，侧边栏不会随窗口拉伸而连续缩放
宽度，拉伸窗口时多出/减少的空间全部由中间 3D 视口吸收。唯一的例外是极窄窗口
下的降级保护：`layoutRects()` 用 `min(310, fb_width/3)` 做上限截断，避免窗口
宽度小于 `3*310=930px` 时两侧面板把 3D 视口完全挤没。

### 验证结果

- `colcon build --symlink-install`（全工作空间 6/6 包）零警告通过。
- 真实桌面会话（`:0`，非 headless）手动截图验证：两侧面板完全落在
  `1280x900` 窗口边界内，不再溢出；点击 `thr_N1` 滑条后其数值从 `0`
  正确变为点击位置对应的 `34.4`，随后依次点击 `thr_N2..N4` 并观察到
  `Pad TL/TR/BL/BR` 遥测从全零变为 `12.5/12.5/55.3/55.3 N`（ROV 被压向壁面
  产生真实接触反力），证明侧边栏滑条点击已能正确驱动物理仿真。

---

## Phase 2.8（2026-10-15）：坐标系尺寸修复、调试面板增强、操作文档

### 目标

Phase 2.7 之后进一步实际操作 MuJoCo 窗口发现："Coord Frames" 显示的坐标系
标记依然偏大；同时希望借此机会为后续 ROV 仿真调试补充更多面板功能，并把
鼠标/键盘操作整理成一份清晰的 README 参考文档。

### 修复与新增内容

1. **坐标系标记整体缩小 `1/50`**（`adhere_mujoco_hardware.cpp::configure()`）：
   在 `mj_loadXML` 之后、而不是修改 `mjcf/rov_wall.xml`（该文件按约定与只读
   参考仓库保持字节一致）直接覆写
   `model_->vis.scale.framelength = 0.02`、`framewidth = 0.002`
   （MuJoCo 默认为 `1.0`/`0.1`）。因为 `vis.scale.*` 是纯可视化参数、不参与
   任何物理计算，这个运行时覆写不影响数值一致性契约。
2. **修复 "Reset" 按钮从未真正生效的潜藏 bug**：`buildUi0()` 早在 Phase 2.6
   就给 Reset 按钮分配了 `userid=kUidReset`，但 `onMouseButton()`/
   `onCursorPos()` 调用 `mjui_event()` 后从未检查返回的 `mjuiItem*`，导致
   按钮即使能被正确命中（Phase 2.7 修复点击坐标翻转之后）也不会触发任何
   动作。新增统一的 `handleUiAction(int userid)` 分发函数，在两处调用点都
   接住 `mjui_event()` 的返回值并分发。
3. **左侧边栏新增调试控件**：`Step Once`（`S` 键等价，暂停时单步前进
   `2.0 ms`，通过新增的 `MujocoVisBridge::step_once_request` 原子标志传给
   `AdhereMujocoHardware::write()`，仅消费一次）、`Reset Camera View`
   （一键恢复默认相机参数）、`Center of Mass`/`Inertia Box` 显示开关
   （`mjVIS_COM`/`mjVIS_INERTIA`，用于核对浮力作用点与惯量箱是否符合预期）。
4. **右侧边栏新增 `Zero All` 按钮**：一键把 8 个推力器滑条清零。
5. **遥测浮层新增机体角速度行** `Body AngVel (wx wy wz)`（取自 MuJoCo
   自由关节 `qvel[3:6]`，机体系），便于单独观察翻滚/失稳趋势。
6. **README 补全"鼠标与键盘操作一览"**：三张表格分别列出 3D 视口鼠标操作、
   全局键盘快捷键、左/右侧边栏每个控件的作用，替换此前零散分布的描述段落。

### 验证结果

- `colcon build --symlink-install`（全工作空间 6/6 包）零警告通过。
- 真实桌面会话截图确认：选中 "Coord Frames: Body" 后，场景中的坐标系标记
  缩小为一个几乎看不清的红色小短线（相比 Phase 2.6 时跨越数个 `0.5m` 网格
  的巨大三色坐标轴，符合"至少缩小到 1/50"的要求）；左右两侧边栏新增的
  `Step Once`/`Reset Camera View`/`Center of Mass`/`Inertia Box`/
  `Zero All` 控件与 `Body AngVel` 遥测行均正确渲染。

---

## Phase 2.9（2026-10-15）：对齐官方 `simulate` UI 功能、面板内容溢出 bug、坐标系缩放定位

### 目标

用户继续实际操作后反馈三点：1）`Coord Frames` 坐标系标记依旧偏大，要求至少
缩小到 `1/50`；2）希望把官方 MuJoCo `simulate` 应用的可视化/调试功能尽量搬到
左右两侧边栏，包括让 ROV 船体透明的功能；3）希望知道坐标系缩放系数可以在
哪里手动调整。

### 坐标系缩放定位（问题 3 的直接答案）

缩放系数是 `mjModel::vis.scale.framelength`/`framewidth` 两个模型级字段，在
`adhere_mujoco_hardware.cpp::configure()` 内、`mj_loadXML` 成功之后被覆写为
`0.02`/`0.002`（MuJoCo 默认 `1.0`/`0.1`）。这是运行时对已加载 `mjModel` 的
覆写，不是编辑 `mjcf/rov_wall.xml`（后者按约定与只读参考仓库保持字节一致，
因为这是纯可视化参数、不参与任何物理计算，运行时覆写不影响数值一致性契约）。

### 新增内容（问题 2）

- 左侧边栏 `Visualize` 新分组：`Transparent Hull`（`mjVIS_TRANSPARENT`，船体
  半透明，直接回应"让 ROV 透明"的请求）、`Joint`/`Actuator`/`Activation`/
  `Camera`/`Light`/`Constraint`/`Contact Point`/`Contact Force`/
  `Contact Split`/`Perturb Force`/`Perturb Object`/`Center of Mass`/
  `Select Point`/`Static Body`/`Inertia Box`/`Scale Inertia w/ Mass`/
  `Auto Connect`，以及扩展为 8 态的 `Coord Frames` 单选（None/Body/Geom/Site/
  Camera/Light/Contact/World）。除 `ROV Overlays`（Pad Contact Forces/Flow
  Velocity Arrow，纯自定义）外，全部直接绑定到 `opt_.flags[...]`/`opt_.frame`，
  不经过中间镜像变量——`mjv_defaultOption()` 已为它们设好 MuJoCo 自身默认值。
- 右侧边栏新增 `Render Flags`（Wireframe/Shadow/Reflection/Skybox/Fog/Haze，
  绑定 `scn_.flags[...]`）、`Label`（17 态单选，绑定 `opt_.label`）、
  `Visibility Groups`（Geom/Site Group 0，绑定 `opt_.geomgroup[0]`/
  `sitegroup[0]`——`rov_wall.xml` 目前所有元素都在默认组 0，故只暴露这一组）、
  新增 `Screenshot` 按钮（左侧边栏，`mjr_readPixels` + 自写 PPM 编码器，保存至
  `/tmp/adhere_mujoco_screenshot_<ms>.ppm`，终端打印路径）。
- 修复一个潜藏 bug：`Reset` 按钮自 Phase 2.6 起就被赋予了 `userid`，但
  `onMouseButton()`/`onCursorPos()` 从未检查 `mjui_event()` 的返回值，点击
  坐标翻转修好后它仍然静默无效。新增统一的 `handleUiAction(int userid)`
  分发函数，在两处调用点都接住返回值。

### 面板内容溢出 bug 的定位与修复

初次加满全部新控件后（单侧约 45 个条目），左侧边栏从第一次渲染起就固定显示
在"Visualize"小节中部（"Camera"开始），`Simulation`/`Wave Environment`/
`ROV Overlays` 等更靠前的小节完全不可见；鼠标滚轮上下滚动 100+ 格均无效果。
排查过程：
- 确认不是 y 坐标翻转问题（Phase 2.7 已修）：手动把 `ui0_.scroll`/`ui1_.scroll`
  在 `layoutRects()` 的 `mjui_resize()` 调用后强制复位为 `0`，并新增
  `ui0_scroll_`/`ui1_scroll_` 成员，在 `onScroll()` 里从 `mjui_event()` 的
  计算结果同步，仍未解决——证明这不是"初始滚动位置不对"，而是
  `mjui_resize()` 在内容显著超过一屏高度后，其内部布局/裁剪行为本身变得
  不可靠（不只是 scroll 字段的问题）。
- 定位方法：把新增的两大块内容（`Render Flags`/`Label`/`Visibility Groups`）
  从左侧边栏整体移到右侧边栏，让左侧边栏内容回落到明显小于一屏高度——
  移动后左侧边栏立即恢复从 `Simulation` 正常显示到底，验证了"内容过高"
  确系根因，而非 y 翻转或其他坐标计算错误。
- 右侧边栏移入这些内容后仍有约 2 行的轻微裁切（`thr_N1`/`Thrusters (Manual)`
  标题被遮住一部分），进一步把价值最低的 `Visibility Groups`
  （对当前 MJCF 而言，组 `1..5` 全部为空操作，只有组 `0` 有实际效果）从
  "6 个 Geom Group + 6 个 Site Group 共 12 行"精简为"仅 Geom Group 0 + Site
  Group 0 共 2 行"，右侧边栏恢复完全正常显示。
- 结论记录在 `mujoco_viewer.hpp` 的 `ui0_scroll_`/`ui1_scroll_` 成员注释中：
  单侧面板内容应控制在明显小于一屏高度的范围内，超过后 `mjui_resize()` 的
  滚动/裁剪行为不可信赖，与是否手动管理 `scroll` 字段无关。

### 验证结果

- `colcon build --symlink-install`（全工作空间 6/6 包）零警告通过。
- 真实桌面会话截图确认：左右两侧边栏现在均从各自第一个小节（`Simulation`/
  `Thrusters (Manual)`）开始完整显示到最后一个小节（`Coord Frames`/
  `Telemetry`），无任何裁切，无需滚动；点击 `Transparent` 复选框后 ROV
  船体渲染为半透明，可透过船体看到内部的接触垫球体与推力器站点标记。

## Phase 2.10（2026-10-15）：6 个隐藏交互 bug 修复、Stonefish 几何驱动水动力引擎、Viewer 调试工具

### 目标

Phase 3（控制器包）启动前的代码审计发现 `adhere_mujoco_hardware` 的
500Hz/60Hz 双线程交互存在 6 个隐藏同步 bug，另有两项结构性需求：(1)
`FossenHydrodynamics` 是硬编码的点质心/平板模型，一旦机体几何换成 CAD 网格
就会失效；(2) `MujocoViewer` 缺少 Rhombic Kern 可视化、实时示波器、相机预设、
慢动作等研究级调试工具。

### 6 个 bug 的根因与修复

1. **`Ctrl+拖拽` 幽灵弹力**：`onMouseButton()` 的 `Ctrl+点击` 分支只设置
   `pert_.active`，从未重新调用 `mjv_initPerturb()`——若 ROV 在上一次双击后
   发生过位移，`pert_.refpos`/`refselpos` 仍是旧位置，导致拖拽时把 ROV 往
   错误方向猛拽。修复：该分支每次触发都重新调用 `mjv_initPerturb()`。同时
   `renderFrame()` 在 `mjv_applyPerturbForce()` 之前从未清零
   `d_vis_->xfrc_applied`，补上清零逻辑，避免残留力泄漏进 `mj_forward()`。
2. **暂停态下 `Step Once`/滑条冻结**：`write()` 原先用物理步数 `step_count_`
   （暂停时不再自增）对 `% 8` 取模决定是否快照到 `MujocoVisBridge`——暂停时
   `step_count_` 冻结，导致 8 次 `Step Once` 中只有 1 次真正更新快照，
   暂停拖滑条同理。新增独立计数 `write_tick_count_`（无论暂停与否每次
   `write()` 都自增），快照门槛改为
   `write_tick_count_ % 8 == 0 || single_step`，"Step Once" 必然当拍可见。
3. **接触垫几何多处硬编码**：`adhere_mujoco_node.cpp` 里的
   `constexpr kPadPosBody` 与 `AdhereMujocoHardware::pad_yz_` 各自独立硬编码
   同一组坐标。新增 `MujocoVisBridge::pad_site_pos_body`（写一次、`configure()`
   之后只读不加锁，与既有 `pad_yz_` 同一假设），`adhere_mujoco_node` 删除
   `kPadPosBody`，统一从该字段读取。
4. **`MujocoViewer::renderFrame()` 单帧 4 次加锁**：原先 1 次 `try_lock`（拷贝
   `qpos`/`qvel`/`ctrl`）+ 3 次独立 `lock_guard`（受力柱状图、流速箭头、
   `drawOverlayTelemetry()` 内部）。改为单次 `try_lock` 临界区内一次性拷贝
   全部所需字段到新增的 `cached_*` 成员，其余 3 处全部改读缓存，不再加锁。
   `grep -c "bridge_->mtx" mujoco_viewer.cpp` 现为 `1`。
5. **"Body Vel" 显示世界系当成机体系**：`drawOverlayTelemetry()` 直接显示
   `d_vis_->qvel[0..2]`，但 MuJoCo 自由关节约定里这是世界系线速度（`qvel[3:6]`
   才是机体系角速度，原本就是对的）。修复：复用该函数里已经为欧拉角显示算出
   的旋转矩阵，做 `R^T * v_world` 再显示。
6. **造浪模式单向同步**：`renderFrame()` 原先只用一个函数局部 `static` 变量
   把 `ui_wave_mode_` 推送给 `bridge_->wave_mode_cmd`，若外部（如
   `/rov/wave_mode_cmd` 话题）改变了 `wave_mode_active`，左侧单选按钮不会
   跟着变。改为两个成员变量 `last_ui_wave_mode_`/`last_external_wave_mode_`，
   先读外部状态并同步进 `ui_wave_mode_`（同时标记"已同步"避免立即原样推回
   造成振荡），再做本地编辑的边沿检测推送。

### Stonefish 几何驱动水动力引擎

`fossen_hydrodynamics.hpp/.cpp` 新增 `HydroModelMode`/`SurfacePatch`/
`HydroGeometryModel`，`FossenHydrodynamics::compute()` 新增 `mode`/`geo`
两个默认参数，`mode == kParametricLumped`（默认）分支与 Phase 2 验证过的
实现逐行保持不变（硬性 parity 要求）。`kStonefishGeometryDriven` 分支：

- `HydroGeometryModel::buildFromMjModel()` 在 `configure()` 时对 `rov_base`
  的全部几何体自动切分表面微元（箭体 box 6 面×4×4=96 个，4 个垫球每个
  6 纬×8 经=48 个，共 288 个原始微元），用空间体素网格按面积加权合并到
  ≤256 个（当前实测确实触发一次合并）；同时支持 `mjGEOM_CYLINDER`/
  `CAPSULE`/`ELLIPSOID`/`MESH`（三角面读取 `mesh_vert`/`mesh_face`）以便
  未来 CAD 网格无需改代码即可接入。
- 包围盒半轴通过逐几何体类型专用半扩展（而非 `geom_rbound` 的等效球半径，
  后者会把 `0.1x0.3x0.2` 的扁平箭体错误膨胀成一个各轴相同的 `0.374` 立方体）
  计算，实测结果 `(0.1, 0.3, 0.2)` 与 MJCF 吻合。
- 用 Lamb 椭球经典附加质量积分（正切换元数值求积，仅在 `buildFromMjModel()`
  时算一次）得到 `added_mass_linear`/`added_mass_angular` 对角张量，实测最薄
  轴（X，半轴 `0.1m`）的附加质量系数因扁平形状显著放大（约 `33.6 kg`，
  超过等效球排水质量 `~25.1 kg`，属扁平体附加质量的已知现象，不与集中参数
  模型的固定 `6kg` 偏置直接比较）。
- 每拍 500Hz 对 ≤256 个微元做分布式压阻/蒙皮摩擦/近壁 Venturi/挤膜阻尼
  积分，再叠加非定常 Froude-Krylov 力与 Munk 回正力矩（见 README 公式）。
- `MujocoVisBridge` 新增 `hydro_mode`/`hydro_patch_count`/`kern_val`/
  `m_kern`/`liftoff_ratio`/`cop_y`/`cop_z`/`flow_acc_world`/`sim_speed_factor`，
  由 `write()` 的快照 tick 一并计算/写入。

### `MujocoViewer` 新增调试工具

- **Rhombic Kern 菱形 + 实时 CoP**：壁面上绘制支撑矩形对角线连接成的菱形
  （由动态解析的 `L_y`/`L_z` 决定，非硬编码）与一个按 `K_val`/脱离壁面状态
  变色（绿/黄/红）的压力中心小球，复用既有受力柱状图/流速箭头的
  `mjv_connector`/`mjv_initGeom` 写法。
- **`mjvFigure` 实时示波器**：右侧新增 "Live Scope" 分组，5 态单选
  （关闭/4 垫受力/Kern 裕度与最小垫力/机体速度 vs 流速/8 推力器指令），
  约 400 帧滚动窗口，渲染在视口右下角 `420x220` 区域。
- **相机预设 + 跟随**：`Default Orbit`/`Wall YZ View`/`Top XY View`/
  `Side XZ View` 4 个预设按钮 + `Camera Follow` 复选框（每帧锁定
  `lookat` 到 ROV 质心，可与任意预设叠加）。
- **慢动作**：`Sim Speed` 单选（`1x/0.5x/0.2x/0.1x`），通过
  `MujocoVisBridge::sim_speed_factor` 把 `adhere_mujoco_node` 物理线程的
  墙钟周期拉长 `1/factor` 倍，仅影响墙钟节奏，不触碰 `mjModel::opt.timestep`。
- **面板分组折叠**：`Visualize`/`Render Flags`/`Label`/`Visibility Groups`
  四个原生 MuJoCo 开关组默认折叠（`mjSECT_CLOSED`），为上述新增控件腾出
  屏幕高度；`Simulation`/`Wave & Hydro Model`/`ROV Overlays`/
  `Thrusters (Manual)`/`Live Scope` 保持展开。

### 验证结果

- `colcon build --symlink-install --packages-up-to simplified_v1_description adhere_mujoco_hardware`
  与全工作空间 `colcon build --symlink-install`（6/6 包）均零编译警告通过。
- `grep -c "bridge_->mtx" src/hardwares/adhere_mujoco_hardware/src/mujoco_viewer.cpp`
  输出 `1`，确认单帧单锁。
- 新增本仓库第一个 gtest（`test/test_fossen_hydrodynamics_geometry.cpp`，
  `ament_cmake_gtest`）：
  - `kParametricLumped` 两组手工推导状态（静止全浸没 / 侧向来流+近壁吸附）
    逐位匹配闭式解（`<1e-9`）；`kStonefishGeometryDriven` 传入 `nullptr`
    几何模型时的防御性回退与 `kParametricLumped` 逐位一致。
  - 对真实 `rov_wall.xml` 构建的 `HydroGeometryModel` 做结构不变量检查
    （微元数 `0 < n <= 256`，法向单位长度，包围盒半轴匹配
    `(0.1,0.3,0.2)` 箭体半扩展，附加质量张量对角且量级合理）与
    `kStonefishGeometryDriven` 模式有限性烟雾测试，全部通过
    （`colcon test --packages-select adhere_mujoco_hardware`：
    `gtest` 5/5 通过；项目既存的 `copyright`/`cpplint`/`flake8`/`pep257`
    风格检查此前即对本仓库全局失败，与本阶段改动无关，未处理）。
- 本阶段未在真实桌面 GLFW 会话中逐项人工复核全部 6 个 bug 修复与新 UI
  控件的视觉效果（环境限制），后续首次桌面会话应按 README 操作表逐项确认，
  尤其是相机 4 个预设的 `azimuth`/`elevation` 符号是否符合预期视角。

## Phase 2.10 审计修复（2026-10-16）：Stonefish 符号/几何 bug 修复、Viewer 可视化与遥测补全

### 目标

Phase 3（控制器包）启动前对 Phase 2.10 引入的 `Stonefish-Mesh` 几何驱动水动力
模型与 `MujocoViewer` 新增调试工具做逐行复核，发现 6 处严重物理符号/几何
bug（均局限于 `kStonefishGeometryDriven` 分支，`kParametricLumped` 零改动）
与 3 处 GUI 可视化/遥测缺口，逐一修复。

### `fossen_hydrodynamics.cpp` 的 6 处物理/几何 bug

1. **Venturi 吸附 + 挤膜阻尼符号反转**：壁面微元外法向 `n_p ≈ [-1,0,0]`；
   原实现 `F_suction_n = -0.5*...*n_p` 在负标量乘负法向下产生 `+X` 排斥力
   （应为 `-X` 吸附力），且 `F_squeeze_n` 对 `v_n` 做了双重取反，造成
   "越靠近壁面越被加速吸入、越远离越被推开" 的正反馈（应为负反馈阻尼）。
   修复：吸附力改为 `+0.5*...*n_p`；挤膜阻尼直接用原始 `v_n`（不再取反）、
   公式改为 `-C_squeeze*A*(h_nom/h)^3*v_n*decay*n_p`，经验证两种模式下
   近壁顺流吸附力 `F_x` 均为负（见下方数值核验）。
2. **`wall_x_offset`（0.10m）重复减去两次**：微元质心 `r_p` 在壁面微元上
   本就已经位于箭体外表面（`x≈-0.10m`），`x_world_p` 已是微元到世界
   `X=0` 壁面的真实距离，原代码仍减去 `wall_x_offset`，导致名义 `0.025m`
   间隙被误算成 `-0.075m`（钳到 `0.001m`）。修复：`h_gap_p = max(x_world_p, 0.005)`，
   不再二次减去偏移量。
3. **`decimatePatches()` 体素合并抹平球面/正交面法向**：原体素键只编码空间
   坐标 `(ix,iy,iz)`，4 个接触垫球（每个 48 微元）各自整体落入同一体素——
   闭合球面法向面积加权求和理论上收敛于零向量，抹平法向的同时把全部球面积
   错误地堆到残余方向上。修复：新增 `normalDirIndex()` 按主导轴+符号分 6 类，
   体素键改为 `(((ix*N+iy)*N+iz)*6+normal_dir`，`N` 从 6 降到 3（`3^3*6=162
   <= 256` 预算），同一体素内的微元必然同法向主导方向，杜绝法向抹平。
4. **背风面二次计重压阻 + 缺少低速线性阻尼/非定常排水惰性/Munk 力矩符号反**：
   - 法向压阻 `Cd_normal` 原来对迎流（`v_n>0`）与背风（`v_n<0`）微元都施加，
     但钝体阻力系数本就定义在单面迎流投影面积上（已含迎流滞止压+背风尾流
     吸力），双面施加等于计重两次——修复为只在 `v_n>0` 时施加法向压阻。
   - 纯二次微元阻力在 `v_rel=0` 处导数为零，定点悬停时丢失低速阻尼刚度——
     补上整体线性阻尼 `-D_lin⊙v_rel,CoM` 与 `-D_ang_lin⊙ω^B`。
   - 非定常流体加速度反力原来只含附加质量项，缺排水体积的 Froude-Krylov
     惯性项——补上 `+rho*V_disp*a_fluid_body`。
   - Munk 回正力矩符号反（`+v×(M_A v)` 应为 `-v×(M_A v)`），且缺旋转附加
     惯量耦合项——改为 `-v_cog×(M_A,lin v_cog) - ω×(M_A,ang ω)`。

### `MujocoViewer`/`MujocoVisBridge`/`adhere_mujoco_node` 的 3 处可视化/遥测缺口

1. **`drawKernDiamond()` 补全灰色支撑矩形 + 脱壁隐藏假 CoP**：新增连接
   `(±L_y,±L_z)` 4 角的灰色外框（在青色菱形之前绘制），直观呈现角三角陷阱
   区；`sum F_N,i <= 1e-3N`（完全脱壁）时不再绘制红色 CoP 小球。
2. **示波器切换通道不清历史**：新增 `last_scope_channel_` 成员，
   `scope_mode_` 变化时清零 `scope_history_count_` 与全部通道缓冲，避免
   `40N` 推力器量级的旧数据残留压扁新通道（如 `0.3m/s` 速度）的自动量程。
3. **补全 `step_work_ms` 纯计算耗时遥测**：`MujocoVisBridge` 新增
   `step_work_ms` 原子字段；`adhere_mujoco_node.cpp` 的 `physics_thread`
   用 `Clock::now()` 包住 `controller_manager->read()/update()/write()`
   做 EMA；`drawOverlayTelemetry()` 显示为 `"%.1f Hz (work: %.3f ms)"`。

### 其余交互审计项（人工走查确认，均已满足或无需改动）

- `onMouseButton()` 的 `Ctrl+Click` 分支每次触发都重新调用
  `mjv_initPerturb()`（Phase 2.10 既有修复，复核仍成立）。
- `onCursorPos()` 的 `Ctrl+拖拽` 默认改为 `mjMOUSE_MOVE_V`/`mjMOUSE_ROTATE_V`
  （沿世界 `Z` 在壁面上下拖拽/旋转 ROV），按住 `Shift` 切换到
  `mjMOUSE_MOVE_H`/`mjMOUSE_ROTATE_H`。
- `AdhereMujocoHardware::write()` 的 `vis_bridge_` 快照条件新增
  `did_reset`/`wave_cmd != 0`/`paused`，使 `Reset`、造浪模式切换、暂停态下
  的每一拍都立即可见，不必等到 `write_tick_count_ % 8 == 0`。
- `grep -c "bridge_->mtx" mujoco_viewer.cpp` 仍为 `1`（单帧单锁契约未破坏）。
- 推力器 `ctrlrange="-40 40"` 已由 MJCF `<general>` 执行器原生限幅；
  本阶段未引入新的 `tau_T` 一阶惯性滞后模型（CLAUDE.md 中标注为
  "optional"，留给 Phase 3 控制器层处理推力器动力学时再决定是否下沉到
  硬件层）。

### 数值符号与量级核验（独立 g++ 编译的核验程序，未并入 gtest）

- 近壁侧向来流（`x_com=0.125m`，`V_y=0.3m/s`）：`Lumped Fx=-4.860N`，
  `Stonefish Fx=-5.018N`，两者均为负（吸附向壁面），符合预期。
- 近壁（`x_com=0.115m`）靠近壁面（`vx_b=-0.05m/s`）vs 远离壁面
  （`vx_b=+0.05m/s`）：`Stonefish Fx` 分别为 `-3.100N`/`-6.935N`，
  靠近时合力更偏 `+X`（阻碍靠近）、远离时更偏 `-X`（阻碍远离），符合
  挤膜阻尼应有的负反馈方向。
- `t=3.2s`（半正弦阵风窗 `[3.0,4.2]s` 内）：`a_fluid_world=[0.45,2.04,0]
  m/s^2` 非零，`Stonefish` 合力/力矩全部为有限值，Froude-Krylov/Munk 力矩
  项参与计算且未发散。
- 单次 `FossenHydrodynamics::compute()` 耗时（`-O2`，200000 次循环平均）：
  `kParametricLumped` 约 `0.19 us`/次；`kStonefishGeometryDriven`
  （66 个微元）约 `4.20 us`/次 —— 远低于 500Hz 的 `2000 us`/拍预算，
  印证两阶段预计算架构（重网格化只在 `configure()` 时算一次约 `1.7ms`）
  确实把 500Hz 热路径开销压到微秒级。
- `colcon build --packages-up-to simplified_v1_description adhere_mujoco_hardware
  --symlink-install` 零警告零错误；`colcon test --packages-select
  adhere_mujoco_hardware` 的 `gtest`（`test_fossen_hydrodynamics_geometry`，
  含原有 2 组 `kParametricLumped` 逐位核验、`nullptr` 回退、几何结构不变量、
  `kStonefishGeometryDriven` 烟雾测试）5/5 全部通过，确认 `kParametricLumped`
  零回归；既存的 `copyright`/`cpplint`/`flake8`/`pep257`/`uncrustify`/
  `xmllint` 风格检查对本仓库全局早已失败，与本次改动无关，未处理。

## Phase 2.11（2026-10-16）：5 个剩余物理/时序 bug 修复

### 目标

对 Phase 2.10 审计修复后的 `adhere_mujoco_hardware` 再做一轮复查，发现并修复
5 个仍未处理的物理与工程缺陷。

### 修复内容

1. **`C_squeeze` 单位不匹配 + 接触垫球体污染水动力外壳切分**：挤膜阻尼公式
   在 Phase 2.10 已改为无量纲比值 `(h_gap_nominal/h_gap_p)^3`（名义间隙处恒为
   `1.0`），但 `C_squeeze` 仍停留在为旧的未归一化 `1/h_gap_p^3`（名义间隙处
   `≈64000`）标定的 `1e-6`，导致名义间隙处挤膜阻尼仅 `≈2.4e-8 N`（约
   6400 万倍过小，近似为零）。修复：`C_squeeze = 64.0`（名义间隙处单板总
   阻尼 `C_squeeze*A_plate=15.36 N·s/m`，物理量级）。同时
   `HydroGeometryModel::buildFromMjModel()` 排除全部 `pad_*` 接触垫球体
   （已由 MuJoCo 自身 Kelvin-Voigt 接触求解器单独建模），原本 `96(箭体)+
   4×48(垫球)=288` 个原始微元超过 `256` 上限触发有损体素合并，垫球 `-X`
   半球微元混入箭体正面板体素、污染正面法向/面积；排除后恰好 `96` 个原始
   微元，无需合并，正面贴壁板精确为 `16` 个微元、`0.24 m^2`、法向严格
   `[-1,0,0]^T`。
2. **Rhombic Kern/脱离遥测被 62.5Hz 节流漏采 + 脱离定义是瞬时而非累积**：
   `kern_val`/`m_kern`/`cop_y/z`/`liftoff_ratio` 原本只在 `write_tick_count_
   % 8 == 0` 的 ~62.5Hz 快照节流块内计算，漏采 `87.5%` 的瞬态 `<5ms` 接触
   事件；`liftoff_ratio` 还只是该拍 4 垫中脱离比例的瞬时值
   （`0/0.25/0.5/0.75/1.0`），不是基准表要求的 500Hz 累积统计。修复：把
   接触力/Kern 判据计算移到每一次真实 `mj_step` 之后（不再受 `%8` 节流
   影响），新增 `total_eval_steps_`/`liftoff_steps_`/`liftoff_events_`
   （上升沿计数）/`corner_trap_steps_` 累积计数器，换算成
   `MujocoVisBridge::liftoff_ratio`/`corner_trap_ratio`/`liftoff_events`/
   `active_pads`；`Backspace` 完整复位或新增的 `Reset Stats` 按钮
   （`reset_stats_request`）均会清零这些计数器。
3. **暂停时 IMU 线加速度被有限差分清零**：`read()` 每个 `500Hz` 周期都会
   用 `(lin_vel_world - prev_lin_vel_world_) / dt` 重新计算并覆写
   `prev_lin_vel_world_`，即使仿真已暂停（`step_count_` 不再前进）也照常
   执行，导致暂停后下一次 `read()` 立刻把 IMU 加速度读数清零。修复：新增
   `last_accel_step_count_`，仅当 `step_count_` 真正变化（物理确实前进了
   一拍）才重新计算/锁存 `last_accel_body_`，暂停期间持续输出锁存值。
4. **缺少推力器 `±40N` 轨道钳位与可选一阶惯性延迟**：`write()` 原来把
   `hw_commands_[i] + manual_ctrl[i]` 不经钳位直接写入 `data_->ctrl`。修复：
   统一钳位到 `[-40.0, 40.0] N`（`CLAUDE.md` §4.3 硬件轨道极限），并新增
   `MujocoVisBridge::enable_thruster_lag` 开关（右侧边栏 "Thruster Lag
   (15ms)" 复选框，默认关闭=直通）：开启后对钳位后的目标值做一阶惯性延迟
   `tau_T=0.015s`（`alpha_T=1-exp(-dt/tau_T)`），且仅在物理确实前进一拍时
   积分，暂停时保持上次值。
5. **相机右键拖拽仍硬编码水平平移 + Kern 菱形/CoP 颜色未分离**：
   `onCursorPos()` 的左键/右键拖拽仍恒为 `mjMOUSE_ROTATE_H`/
   `mjMOUSE_MOVE_H`，无法沿壁面竖直方向平移/环绕相机，除非按住 `Shift`。
   修复：改为 `uistate_.shift ? ..._H : ..._V`，默认竖直变体，`Shift` 切换
   水平变体（与既有 `Ctrl`+拖拽扰动的约定一致）。`drawKernDiamond()` 原来
   让菱形边界本身随 `K_val`/脱离状态变色，与 CoP 球颜色重复、难以区分
   边界与状态点；改为菱形边界固定青色（`{0.1,0.85,1.0}`），CoP 球改为按
   `active_pads`/`kern_val` 变色：绿（4 垫接触且 `K_val<=1.0`）/黄（4 垫
   接触但 `K_val>1.0`，角三角陷阱）/红（任意一垫脱离）。

### 单元测试新增

`test_fossen_hydrodynamics_geometry.cpp` 新增 2 组测试，锁定上述物理修复：

- `FrontPlateAreaAndNormalsExact`：验证 `geo.patches.size()==96`、所有正面
  微元法向严格 `[-1,0,0]^T`、质心 `x` 严格 `-0.10m`、正面总面积严格
  `0.24 m^2`。
- `StonefishVenturiAndSqueezePhysicalSigns`：验证侧向来流下吸附力
  `Fx<-1.0N`；靠近壁面时合力比静止基线高 `>0.5N`（阻碍靠近）、远离壁面时
  低 `>0.5N`（阻碍远离）——锁定 `C_squeeze=64.0` 的物理量级而非早期近零力。

### 验证结果

- `colcon build --packages-up-to simplified_v1_description adhere_mujoco_hardware
  --symlink-install` 零警告零错误。
- `colcon test --packages-select adhere_mujoco_hardware` 的 `gtest`
  （`test_fossen_hydrodynamics_geometry`）7/7 全部通过（原有 5 组 + 本次
  新增 2 组），`kParametricLumped` 数值零回归；既存的
  `copyright`/`cpplint`/`flake8`/`pep257`/`uncrustify` 风格检查与
  `gtest`（未 `source install/setup.bash` 时找不到包共享目录）在未改动的
  `main` 分支上同样失败，与本次改动无关，未处理。

---

## Phase 2.12（2026-10-16）：`adhere_mujoco_hardware` 模型无关化（零重编模型切换）+ 2 项物理细节修复

### 目标

消除 `adhere_mujoco_hardware` 中残留的 5 处与 `simplified_v1_description`/
`rov_wall.xml` 绑定的硬编码假设，使切换到未来的新 ROV 模型包（不同壳体尺寸、
焊接子刚体、质量/浮力、推力器量程）只需改动启动参数与新模型自己的 MJCF，无需
修改或重新编译 `adhere_mujoco_hardware` 的任何 C++ 源码；同时修复 2 项代码
审计中发现的物理细节问题。

### 修复/新增内容

1. **启动时模型来源解析（零重编模型切换）**：新增
   `AdhereMujocoHardware::setModelSourceOverride(pkg, mjcf)` 静态方法（必须
   在 `ResourceManager` 构造——即 `configure()` 同步执行——之前调用，因为此时
   尚无实例可通过 `activeInstance()` 访问），由 `adhere_mujoco_node` 从新增的
   `pkg_description`/`mjcf_file` 节点参数调用；`mujoco_sim.launch.py` 新增
   `pkg_description`/`mjcf_file`/`xacro_file` 三个 launch 参数（默认值分别为
   `simplified_v1_description`/`mjcf/rov_wall.xml`/
   `urdf/simplified_v1.urdf.xacro`，与改动前行为逐位一致）。`configure()`
   解析优先级：静态覆盖 → `<hardware><param>` HardwareInfo → 硬编码默认值。
2. **焊接子刚体递归 + 根刚体名称回退链**：`HydroGeometryModel::
   buildFromMjModel()` 不再假设 `root_body_id` 的所有水动力几何都是其直属
   `<geom>`，改为单次正向遍历 `mjModel`（利用 `body_parentid[b] < b` 的
   MuJoCo 保证）判定每个刚体是否通过一条全程零自由度
   （`body_jntnum==0`，真正"焊接"）的父链归属于 `root_body_id` 的刚性壳体
   子树，并沿该父链累乘 `body_pos`/`body_quat` 把子刚体的每个 `<geom>`
   复合变换进 `root_body_id` 坐标系后再送入切分函数。`AdhereMujocoHardware::
   configure()` 的根刚体解析新增回退链：`"rov_base"` → `"base_link"` →
   模型中第一个拥有 `mjJNT_FREE` 自由关节的刚体。
3. **推力器轨道上限动态解析**：`configure()` 为 8 个推力器分别从
   `model_->actuator_ctrlrange` 读取量程（未声明则回退 `±40N`），写入新增的
   `MujocoVisBridge::thruster_ctrl_min/max`（一次写入、全程只读，与
   `pad_site_pos_body` 同一模式）；`write()` 的钳位与 `MujocoViewer::
   buildUi1()` 右侧边栏 8 个滑条的量程字符串均改为读取这对数组，不再硬编码
   `±40.0N`/`"-40 40"`。
4. **MJCF `<custom><numeric>` 水动力参数覆盖**：新增
   `HydroParams::applyMjModelOverrides(m, params)`，扫描 MJCF 的
   `<custom><numeric name="hydro_..." data="..."/>` 条目并覆盖 14 个可选字段
   （`hydro_vehicle_mass`/`hydro_augmented_mass`/`hydro_displaced_volume`/
   `hydro_cob_offset_body`/`hydro_z_surface`/`hydro_D_lin`/`hydro_D_quad`/
   `hydro_D_ang_lin`/`hydro_D_ang_quad`/`hydro_C_venturi`/
   `hydro_h_gap_nominal`/`hydro_A_plate`/`hydro_C_squeeze`/`hydro_Cd_normal`/
   `hydro_Cf_tangent`），MJCF 未声明的字段保持 `HydroParams` 默认值不变。
   `FlowEnvironmentPlugin` 新增 `mutableHydroParams()` 虚接口（供
   `configure()` 的这一次性覆盖调用使用，其余调用点仍走原有的只读
   `getHydroParams()`），`configure()` 对 `calm_plugin_`/`gust_plugin_`
   各调用一次。
5. **2 项物理细节修复**：
   - `FossenHydrodynamics::compute()` 的 `kStonefishGeometryDriven` 分支，
     壁面吸附项的 `v_tan_sq_p` 由手写的 `v_t.y()^2+v_t.z()^2`（隐含假设贴壁
     微元法向恒为 `[-1,0,0]^T`）改为 `v_t.squaredNorm()`——对平板场景数值
     完全相同，对未来弯曲 CAD 壳体的任意法向方向微元才是严格正确的切向
     速度模长平方。
   - `AdhereMujocoHardware::write()` 的 `did_reset` 分支新增
     `applied_thrust_.fill(0.0);`，避免 `enable_thruster_lag` 开启时复位前
     残留的一阶惯性延迟状态泄漏进复位后的第一拍。

### 单元测试新增

`test_fossen_hydrodynamics_geometry.cpp` 新增
`TEST(ModelAgnosticHydro, WeldedChildBodyRecursionAndCustomNumerics)`：
构造一个内存中（经临时文件加载）的最小 MJCF，包含一个自由浮动根刚体
`base_link`、一个零自由度焊接子刚体 `fairing_child`（各带一个 `0.1m`/
`0.05m` 半边长的 box 几何）、一条 `<custom><numeric
name="hydro_vehicle_mass" data="35.0"/>` 与一个 `ctrlrange="-60 60"` 的
推力器。验证：（a）`buildFromMjModel()` 递归产出 `96+96=192` 个微元（均在
`256` 上限内，无有损合并），且至少一个微元质心在 `root` 坐标系下
`x>0.15m`——只有递归穿过焊接子刚体才可能出现（`fairing_child` 相对
`base_link` 偏移 `0.2m`）；（b）`applyMjModelOverrides()` 把
`vehicle_mass` 覆盖为 `35.0`，而 MJCF 未声明的 `augmented_mass` 保持
默认值 `26.0` 不变。

### 验证结果

- `colcon build --packages-up-to simplified_v1_description adhere_mujoco_hardware
  --symlink-install` 零警告零错误。
- `colcon test --packages-select adhere_mujoco_hardware` 的 `gtest`
  （`test_fossen_hydrodynamics_geometry`，`source install/setup.bash` 后
  直接运行二进制）8/8 全部通过（原有 7 组 + 本次新增 1 组），原有
  `kParametricLumped` 逐位核验与 `rov_wall.xml` 的 `96` 个微元/`0.24 m^2`
  正面板面积零回归；既存的
  `copyright`/`cpplint`/`flake8`/`pep257`/`uncrustify`/`xmllint` 风格检查
  在未改动的 `main` 分支上同样失败，与本次改动无关，未处理。
- `ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py headless:=true
  use_rviz:=false`（默认参数）与追加显式
  `pkg_description:=simplified_v1_description mjcf_file:=mjcf/rov_wall.xml`
  两种调用方式均成功 `configure()` 并输出"HydroGeometryModel built with 96
  surface patches"，确认启动时模型来源解析的默认路径与显式覆盖路径行为
  一致、零回归。

## Phase 2.13（2026-10-16）：3 项跨模块修复、实时水动力分项遥测、五级水动力精确性 V&V 体系

### 目标

在锁定 Phase 2 转入 Phase 3（控制器包）之前做最后一轮跨模块审计：修复 3 处
隐藏的模块间状态不同步问题，为 `FossenHydrodynamics::compute()` 增加可选的
逐项力/力矩分解输出（供 `MujocoViewer` 实时示波器与自动化 V&V 测试共同使用），
并补齐一套面向 *Elsevier Ocean Engineering* 发表级精确性要求的五级水动力精确性
验证（5-Tier V&V）测试体系。

### 3 项跨模块修复

1. **`MujocoViewer` 根刚体 ID 硬编码**：`MujocoViewer::MujocoViewer()` 此前
   独立执行一遍硬编码的 `mj_name2id(model_, mjOBJ_BODY, "rov_base")`，与
   `AdhereMujocoHardware::configure()` 自己的回退链（`"rov_base"` →
   `"base_link"` → 第一个 `mjJNT_FREE` 刚体）各自为政——换成根刚体命名为
   `"base_link"` 的新模型时，`MujocoViewer::body_id_` 会静默变成 `-1`，导致
   相机跟随/流速箭头/Rhombic Kern 菱形/`Ctrl+拖拽`扰动注入全部失效。新增
   `MujocoVisBridge::root_body_id`（一次写入、全程只读，与
   `pad_site_pos_body` 同一模式），由 `configure()` 在自己解析出
   `body_id_` 之后立即写入；`MujocoViewer` 构造时改为优先读取这个值，只有
   在 `bridge_` 尚未解析出有效值时才回退到原来的硬编码查找（防御性兜底，
   正常路径下不会触发）。
2. **`buildFromMjModel()` 未处理非零质心偏移 `body_ipos`**：
   `AdhereMujocoHardware::applyHydrodynamics()` 始终以根刚体的 CoM
   （`d->xipos`）为参考点求水动力微元的运动学/杠杆臂，但
   `HydroGeometryModel::buildFromMjModel()` 切分出的微元质心此前是相对刚体
   关节原点（`<geom>`/`<body>` 位姿复合后的坐标，等价于 `d->xpos` 系）而非
   CoM。`rov_wall.xml` 声明 `<inertial pos="0 0 0"/>`，两者正好重合，故这
   是一个此前零症状的潜伏 bug。修复为：切分完成后，所有微元质心与包围盒
   统一减去 `m->body_ipos[root_body_id]`，使其严格以 CoM 为原点——对现有
   模型是零改动（减去零向量），只在未来 CAD 模型声明非零 `<inertial pos>`
   时才会产生实际差异。
3. **`kStonefishGeometryDriven` 缺失二次角阻尼**：该分支此前只叠加了线性
   角阻尼 `-D_ang_lin ⊙ ω^B`，遗漏了 `kParametricLumped` 分支一直具备的
   二次项 `-D_ang_quad ⊙ (ω^B ⊙ |ω^B|)`。由于微元杠杆臂很小
   （`r_p~0.15m => r_p^3~0.0034 m^3`），仅靠微元法向阻力积分在高角速度下
   会低估钝体旋转涡脱阻尼一个数量级。补齐后两种模式的角阻尼公式完全对称。

### 实时水动力分项遥测：`HydroWrenchBreakdown`

`fossen_hydrodynamics.hpp` 新增 `struct HydroWrenchBreakdown`（`f_drag_body`/
`f_venturi_body`/`f_squeeze_body`/`f_fk_am_body`/`f_buoy_world`/
`t_munk_body`，外加一个供 Tier 4 被动性校验使用的耗散性旋转阻尼分量
`t_drag_body`），`FossenHydrodynamics::compute()` 新增一个默认 `nullptr` 的
可选输出指针参数 `breakdown`（不影响任何既有调用点的签名/行为），非空时在两种
模式下都逐项回填。`MujocoVisBridge` 新增 4 个原子量
（`hydro_f_drag_y`/`hydro_f_venturi_x`/`hydro_f_squeeze_x`/
`hydro_f_fk_am_y`），由 `AdhereMujocoHardware::applyHydrodynamics()` 每拍
500Hz 写入；`MujocoViewer::buildUi1()` 的 "Live Scope → Scope Plot" 单选新增
第 6 个选项 `Hydro Breakdown (N)`，可在运行时实时查看各项水动力效应的真实
牛顿量级。

### 五级水动力精确性验证体系（5-Tier V&V）

`test_fossen_hydrodynamics_geometry.cpp` 新增 5 个测试，覆盖 Lamb 球解析极限
（`HydroVV_Tier1`）、微元-集总模型退化精确等价（`HydroVV_Tier2_
PatchToLumpedExactEquivalence`）、倾斜/旋转下的解析曲面积分基准
（`HydroVV_Tier3_AnalyticalPitchSqueezeAndVenturiTorque`）、热力学被动性/
Munk 无功定律（`HydroVV_Tier4_ThermodynamicPassivityAndWorklessMunk`，200
组固定种子伪随机状态）、闭环 MuJoCo 动态轨迹（`HydroVV_Tier5_
ClosedLoopTerminalVelocityAndRollStability`，实际 `mj_step` 仿真）。
方法论细节见 `adhere_mujoco_hardware/README.md`"数值校验与五级水动力精确性
验证体系（5-Tier V&V）"一节。Tier 1 的解析极限校验过程中发现
`buildFromMjModel()` 的旋转附加惯量 `I_A` 启发式公式
（`0.2*m_A_i*(其余两轴半长²之和)`）在 `a=b=c`（球体）处不收敛于 Lamb 的
经典零值结论，补充了一个球形特判（三轴半长近似相等时直接返回
`I_A=Zero()`），非球形壳体的既有启发式公式不变。

### 验证结果

- `colcon build --packages-up-to simplified_v1_description adhere_mujoco_hardware
  --symlink-install` 零警告零错误（唯一出现的 `CMake Warning (dev)` 来自
  `FindOpenGL.cmake` 的策略提示，与本次改动无关，`main` 分支上同样存在）。
- `colcon test --packages-select adhere_mujoco_hardware` 的 `gtest`
  （`test_fossen_hydrodynamics_geometry`）**13/13 全部通过**（原有 8 组
  + 本次新增 5 组 5-Tier V&V），原有 `kParametricLumped` 逐位核验、
  `rov_wall.xml` 的 `96` 个微元/`0.24 m^2` 正面板面积、焊接子刚体递归等
  既有测试零回归。
- `ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py headless:=true
  use_rviz:=false` 成功 `configure()` 并输出 "HydroGeometryModel built with
  96 surface patches"，确认 3 项修复未破坏启动路径。
