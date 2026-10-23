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

## Phase 2.14（2026-10-16）：水动力学精度审计修复、壁面邻近附加质量增强、500Hz 逻辑测试覆盖补齐

### 目标

对 `adhere_mujoco_hardware` 全量源码（`adhere_mujoco_hardware.cpp/.hpp`、
`fossen_hydrodynamics.cpp/.hpp`、`flow_environment_plugin.cpp/.hpp`、
`mujoco_viewer.cpp/.hpp`、`adhere_mujoco_node.cpp`、构建/测试配置）做一次
独立于具体 Phase 任务的逐行审计（线程安全、内存管理、ROS 2 Foxy API 兼容性、
MuJoCo 数据/指针安全、500Hz 数值稳定性），并对审计中确认的问题逐一修复，
同时实现一项水动力学精度增强、补齐此前完全缺失的 500Hz 逻辑单元测试覆盖。

### 4 项审计修复

1. **`configure()` 错误路径 `model_` 内存泄漏**：`body_id_`/pad 触摸传感器/
   pad site/推力器 actuator 这 4 处"MJCF 解析失败"早退分支，此前
   `mj_loadXML()` 已成功加载的 `model_` 未被释放就直接 `return ERROR`，
   依赖"对象最终被析构"这一未经验证的外部生命周期假设。现在每个分支
   在返回前显式 `mj_deleteModel(model_); model_ = nullptr;`，使错误路径
   自身资源安全。
2. **两种水动力学模式的近壁间隙下限不一致且未文档化**：`kParametricLumped`
   用 `0.001m`（Python 参考逐位对等锁定的一部分，不可更改），
   `kStonefishGeometryDriven` 的微元循环用 `0.005m`，此前是两个未命名、
   未解释的裸常量。拆成 `fossen_hydrodynamics.cpp` 文件作用域内两个显式
   命名常量 `kWallGapFloorComRef`（CoM 参考点）/`kWallGapFloorPatchRef`
   （微元表面参考点），数值均未改动（避免打破前者的 Python 对等锁定、
   或改变后者已被 V&V 套件验证过的近壁挤膜阻尼上限），但现在是有据可查的
   设计决策而非隐性分歧。
3. **`HydroGeometryModel` 附加质量张量硬编码 `rho_water=1000.0`，与
   `FlowEnvironmentPlugin` 实际配置的密度脱节**：`buildFromMjModel()`
   新增 `rho_water` 参数（默认 `1000.0`，向后兼容每一个既有调用点/测试），
   `AdhereMujocoHardware::configure()` 改为显式传入
   `calm_plugin_->getHydroParams().rho_water`，使附加质量张量与
   `compute()` 内非定常 Froude-Krylov 力项共享同一个"环境水密度"口径。
4. **Rhombic Kern 的 `L_y`/`L_z` 无下限保护，退化几何会产生 `NaN`**：
   `write()` 内联的 CoP/`K_val`/掀离/角落陷阱分类逻辑提取为新文件
   `include/adhere_mujoco_hardware/rhombic_kern.hpp` 的纯函数
   `classifyRhombicKern()`（不依赖 `mjModel`/`ResourceManager`），为
   `L_y`/`L_z` 加了 `1e-6m` 下限保护，彻底消除"未来 MJCF 若声明共线
   （退化）贴垫几何，`kern_val` 会除零变成 `NaN/Inf` 并污染
   `MujocoVisBridge` 全部下游消费者"的隐患。同一文件还把 ESC 死区
   （`applyThrusterDeadzone`）与一阶惯性迟滞滤波单步
   （`applyThrusterLagStep`）一并提取为纯函数，`write()` 中改为调用这三个
   函数，行为完全不变。

### 水动力学精度增强：壁面邻近附加质量

`kStonefishGeometryDriven` 分支新增基于经典势流镜像法（Lamb, Newman
*Marine Hydrodynamics* §4.14）的壁面邻近附加质量放大：法向轴
`1+3(a/2h)³`、切向轴 `1+(3/16)(a/h)³`（`a` 取 `bbox_half_extents` 对应
分量，`h` 为 CoM 到壁面间隙），应用于 `F_fk_am_body`/`T_munk_body` 内部
使用的附加质量张量（不回写 `geo.added_mass_linear` 本身，仅在
`compute()` 内按当次调用的 `h_gap` 局部缩放），钳位在 `6.0` 倍放大以内
防止镜像级数在 `h→0` 处的理想流体奇异发散。旋转附加惯量未做同等修正
（需要更高阶多极镜像分析，留作已记录的未来增强项，不虚构未经验证的
公式）。详见 `adhere_mujoco_hardware/README.md` "双模式水动力引擎"一节。

### 单元测试新增

- `test/test_fossen_hydrodynamics_geometry.cpp` 新增 2 组回归测试：
  `RhoWaterParameterizationScalesAddedMassLinearly`（验证 `rho_water`
  对两个附加质量张量严格线性缩放、对几何量零影响）、
  `WallProximityAddedMassAmplifiesNearWallAndIsBounded`（验证远壁退化、
  近壁单调放大、贴壁有界钳位三段行为）。
- 新建 `test/test_rhombic_kern_and_thruster_model.cpp`（9 个用例），覆盖
  `rhombic_kern.hpp` 的 Rhombic Kern 分类（均匀受载/单垫掀离/角落陷阱/
  全脱离默认值/**退化共线贴垫不产生 `NaN`**）与 ESC 死区/一阶迟滞滤波的
  收敛性、平衡点保持、正负对称死区行为——这是 `adhere_mujoco_hardware.cpp`
  500Hz 主循环逻辑第一次获得单元测试覆盖（此前只有
  `fossen_hydrodynamics.cpp` 的纯函数被测试）。头文件无 MuJoCo 依赖，
  `CMakeLists.txt` 新增的 `ament_add_gtest` 目标无需链接 `MUJOCO_LIBRARY`。
- `package.xml` 补充 `simplified_v1_description` 的 `test_depend`
  （此前完全未声明，隔离式 `colcon test --packages-select
  adhere_mujoco_hardware` 在该包尚未构建时会直接运行期异常失败）。确认
  `simplified_v1_description` 自身 `package.xml` 对本包零依赖声明，这是
  单向边而非环，不违反现有"避免双向 `depend` 构建环"的约定。

### 验证结果

- `colcon build --packages-up-to adhere_mujoco_hardware` 零警告零错误
  （`simplified_v1_description` 因新增 `test_depend` 被正确提前构建）。
- `colcon test --packages-select adhere_mujoco_hardware` 的两个 `gtest`
  目标合计 **25/25 全部通过**：`test_fossen_hydrodynamics_geometry`
  16/16（原有 14 组零回归 + 本次新增 2 组），`test_rhombic_kern_and_
  thruster_model` 9/9（全新）。既存的
  `copyright`/`cpplint`/`flake8`/`pep257`/`uncrustify` 风格检查在改动前的
  14 个文件上同样失败（每个文件均缺版权头，与本次改动无关），新增的
  `rhombic_kern.hpp`/`test_rhombic_kern_and_thruster_model.cpp` 仅复现
  同一"缺版权头"问题，未引入任何新增类别的风格问题，未处理。

---

## Phase 3.1（2026-10-17）：FSM 控制框架（`StatePassive`/`StateDepthHold`）、`AdhereKamrController` 插件、FSM 终端控制台

### 目标

在 Phase 2 验证过的 500Hz `AdhereMujocoHardware` 被动漂流物理闭环之上，首次闭合
控制回路：搭建 `adhere_control_msgs`/`adhere_controller_common`/
`adhere_kamr_controller`/`adhere_keyboard_input` 四个包协作的模块化 FSM 流水线
（`StatePassive` 零输出安全态 + `StateDepthHold` 闭环 PID 定深），并提供交互式
终端控制台用于运行时切换状态/更新定深目标。

### 包 1：`src/interfaces/adhere_control_msgs`

新增 `msg/FsmCommand.msg`（`std_msgs/Header header`；`int32 command` 常量
`NO_CHANGE=0, PASSIVE=1, DEPTH_HOLD=2`；`float64 target_z, target_y, target_yaw`，
后两者为未来平面站位保持/航向控制预留），`CMakeLists.txt` 新增对应的
`rosidl_generate_interfaces()` 调用。

### 包 2：`src/controllers/adhere_controller_common`

- `common/enumClass.h`：`FSMStateName{INVALID, PASSIVE, DEPTH_HOLD}`、
  `FSMMode{NORMAL, CHANGE}`。
- `common/mathTypes.h`/`mathTools.h`：Eigen 类型别名（`Vec3/Vec6/Vec8/Mat68/
  Mat86/RotMat`）与 `quatToRotMat`/`rotMatToRPY`（ZYX 欧拉角提取）/`saturation`。
- `CtrlInterfaces.h`：`ControlInputs`（`command`/`target_z`/`target_y`/
  `target_yaw` 均为 `std::atomic<T>`，供 ROS executor 线程写、500Hz 物理线程
  无锁读）+ 固定顺序的 8 推力器指令/8 推力器状态/10 IMU/10 Odom/8 pad-contact
  `LoanedCommandInterface`/`LoanedStateInterface` 引用容器。
- `FSM/FSMState.h`：抽象基类（`enter()`/`run(time, period)`/`exit()`/
  `checkChange()`），移植自 `quadruped_ros2_control` 设计但改用本项目自己的
  `CtrlInterfaces`/`FSMStateName`。
- `FSM/StatePassive.{h,cpp}`：`enter()`/`run()` 均将 8 个推力器指令置零；
  `checkChange()` 在 `command==2` 时迁移至 `DEPTH_HOLD`。
- `FSM/StateDepthHold.{h,cpp}`：世界坐标系 Z 向 PID（`Kp=180, Ki=95, Kd=62`，
  抗饱和积分钳位 `±180/95N`，测量值微分避免设定值突变时的微分冲击）+ 滚转/
  俯仰/偏航姿态阻尼（`Kp_att=6, Kd_att=4`，纯阻尼/弱回正，因
  `rov_wall.xml` 的切向推力器安装点 `X=0.0` 与质心 `X` 坐标重合、不存在
  `CLAUDE.md` §5 所述 `X=-0.10m` 杠杆臂，故无需额外的寄生俯仰/偏航力矩补偿项）；
  期望力矩经逐项从 `rov_wall.xml` 实际几何（角点 `(y,z)=(±0.25,±0.15)m`、法向
  推力器 `gear="-1 0 0 0 0 0"`、切向推力器沿各自 `zaxis`）推导出的精确 `6x8`
  刚体力/力矩分配矩阵 `B` 的 Moore-Penrose 伪逆（`Eigen::
  CompleteOrthogonalDecomposition::pseudoInverse()`）映射到 8 个推力器指令，
  逐分量钳位至 `[-40,40]N`。`integral_z_` 仅在 `enter()`（即
  `PASSIVE -> DEPTH_HOLD` 迁移）清零，运行时更新 `target_z` 不触发 `enter()`，
  故不重置积分器。
- `CMakeLists.txt` 由 Phase 1 的全量注释占位改为真实构建
  `adhere_controller_common` 共享库（`src/FSM/StatePassive.cpp` +
  `src/FSM/StateDepthHold.cpp`），`ament_export_targets`/
  `ament_export_libraries` 导出供 `adhere_kamr_controller` 消费。

### 包 3：`src/controllers/adhere_kamr_controller`

- `include/adhere_kamr_controller/adhere_kamr_controller.hpp` +
  `src/adhere_kamr_controller.cpp`：`AdhereKamrController`
  （`controller_interface::ControllerInterface` 插件，ROS 2 Foxy 精确签名：
  `init(const std::string&)`、`update()` 无参、`on_configure/on_activate/
  on_deactivate/on_cleanup/on_error/on_shutdown` 均为
  `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn`）。
  `on_configure()` 订阅 `/rov/fsm_command`，原子写入 `ControlInputs`；
  `on_activate()` 按固定全名（如 `thr_N1/effort`、`imu_sensor/orientation.w`、
  `rov_odom/pos.x`、`pad_contact/pad_TL/force`）在 `command_interfaces_`/
  `state_interfaces_` 中逐一查找装入 `CtrlInterfaces`（不假设
  `controller_manager` 内部装配顺序等于请求顺序），默认启动于 `PASSIVE`；
  `update()` 用 `get_node()->now()` 与上次调用时刻的差值得到实际 `period`
  （而非假设恒定 `1/frequency_`），驱动 FSM 的 `run()`/`checkChange()`/
  `enter()`/`exit()` 切换循环。
- `adhere_kamr_controller.xml` 取消 Phase 1 的注释占位，正式导出插件。
- `CMakeLists.txt` 由全量注释改为真实构建该插件库并
  `pluginlib_export_plugin_description_file`；新增
  `config/controllers.yaml`（`controller_manager` 参数服务器描述，供未来
  spawner 式启动参考——当前编程式加载路径不读取它）。

### 包 4：`src/commands/adhere_keyboard_input`

新增 `src/fsm_console_node.cpp`：订阅 `/rov/odom` 在独立 executor 线程持续刷新
显示的当前 Z，主线程阻塞 `std::cin` 读取菜单选择（`1`=`PASSIVE`，`2`=提示输入
目标深度后发 `DEPTH_HOLD`，`q`=退出控制台），发布 `FsmCommand` 至
`/rov/fsm_command`。`CMakeLists.txt` 新增 `fsm_console_node` 可执行目标。

### 包 5：`src/hardwares/adhere_mujoco_hardware`（集成，非新增）

- `adhere_mujoco_node.cpp`：在物理线程启动前调用
  `controller_manager->load_controller(name, type)` + `configure_controller(name)`
  加载/配置 `adhere_kamr_controller`；`switch_controller(...)`
  （真正激活）则推迟到物理线程**启动之后**才调用。
  **关键发现**：ROS 2 Foxy 的 `ControllerManager::switch_controller()` 会阻塞
  等待下一次 `ControllerManager::update()` 真正执行切换（两阶段设计：
  `switch_controller()` 只是登记待切换请求，`update()` 才实际调用
  `on_activate()`/清除待切换标志）；若在任何线程开始调用 `update()` 之前就
  调用 `switch_controller()`，默认的无限超时会导致永久死锁——本次实现中
  最初把三步全部放在物理线程启动之前，复现了这个死锁（进程挂起、
  `/rov/odom` 从不出现），之后通过把 `switch_controller()` 单独移到
  `std::thread physics_thread(...)` 构造之后解决。
- `launch/mujoco_sim.launch.py`：新增 `spawn_fsm_console`（默认 `true`）
  launch 参数，用 `ExecuteProcess` + `bash -c` 探测 `gnome-terminal`/`xterm`
  （均不存在时打印手动运行提示而非让整个 launch 失败）自动弹出
  `fsm_console_node` 终端窗口。
- `package.xml` 新增 `controller_manager_msgs`（`depend`，
  `SwitchController::Request::STRICT` 常量）、`adhere_kamr_controller`/
  `adhere_keyboard_input`（`exec_depend`，运行时通过 pluginlib ament-index
  插件清单发现/通过 `ros2 run` 启动，非头文件/链接依赖，不构成与
  `adhere_mujoco_hardware` 的构建环）。

### 推力分配矩阵 `B` 的推导（`StateDepthHold.cpp`）

`rov_wall.xml` 的 8 个推力器全部安装在 `X=0.0`（与质心 `X` 坐标相同，无杠杆臂），
4 个角点 `(y,z) ∈ {(0.25,0.15),(-0.25,0.15),(0.25,-0.15),(-0.25,-0.15)}`：
法向推力器（`thr_N1..N4`，方向 `-X_B`）贡献 `Fx=-u, Ty=-z*u, Tz=y*u`；切向推力器
（`thr_T5..T8`，方向沿各自 `zaxis` 在 Y-Z 平面内 `±cos30°/±sin30°`）贡献
`Fy=dy*u, Fz=dz*u, Tx=(y*dz-z*dy)*u`（`Ty=Tz=0`，因杠杆臂的 `X` 分量为零）。
`rank(B)=6 < 8` 列，欠定系统的最小范数（伪逆）解在无饱和时精确复现期望的
6 维力/力矩，不存在结构性稳态残差。

### 闭环验证结果

`colcon build --symlink-install`（全部 6 个包）与隔离式
`colcon build --packages-select <pkg>`（逐包）均 **零警告零错误**。
`colcon test --packages-select adhere_mujoco_hardware` 的两个 `gtest` 目标
**35/35 全部通过**，对本阶段改动（`adhere_mujoco_node.cpp`/
`CMakeLists.txt`/`package.xml`）零回归（既有 style-lint 失败——`copyright`/
`cpplint`/`flake8`/`pep257`/`uncrustify`——是 Phase 2.14 已记录的"缺版权头"
预先存在问题，与本阶段改动无关）。

用 `rclpy` 脚本化驱动 `ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py
headless:=true use_rviz:=false spawn_fsm_console:=false` 跑完整 FSM 生命周期
（经过 `Kp_z=180, Ki_z=95, Kd_z=62` 的增益整定，初版 `Kp_z=120, Ki_z=15,
Kd_z=55` 在 6s 窗口内收敛至 25-30mm 残差，不满足 `<5mm` 契约，提升积分增益后
达标）：

| 阶段 | 目标 | 耗时 | 终值 `z` | 残差 |
|---|---|---|---|---|
| `PASSIVE`（默认启动） | 自由漂流 | 2s | 单调上升（净浮力驱动） | — |
| `DEPTH_HOLD` | `z_des=0.40m` | 6s | `0.3970m` | `3.0mm` ✅ `<5mm` |
| `DEPTH_HOLD`（运行时重定目标，同一状态不重置积分器） | `z_des=0.75m` | 6s | `0.7539m` | `3.9mm` ✅ `<5mm` |
| `PASSIVE`（切回） | 自由漂流 | 2s | 恢复单调上升 | — |

`fsm_console_node` 手动交互测试（管道注入 `1`/`2`+深度值/`q`）确认菜单渲染、
`Current Z` 实时刷新（ROS 2 发现延迟后的首次 `/rov/odom` 到达，数秒量级，
符合预期）、`FsmCommand` 发布三条路径均正确工作。

## Phase 3.1.1（2026-10-17）：均匀整数抽取多速率（500Hz→250Hz/25Hz）、`adhere_mujoco_hardware`/`adhere_kamr_controller` 严格包解耦

### 动机

Phase 3.1 为凑出精确 `200Hz`（从 `500Hz` 主时钟）使用了 `[3,3,3,3,3,2,2,2,2,2]`
交替子步调度表（`3` 拍 `6ms` / `2` 拍 `4ms` 交替），给 Pad-Force DOB 等未来的
离散积分器/微分器/低通滤波器引入 `±20%` 采样周期抖动。同时 `adhere_mujoco_node.cpp`
硬编码了 `load_controller("adhere_kamr_controller", "adhere_kamr_controller/
AdhereKamrController")` 与 `switch_controller({"adhere_kamr_controller"}, ...)`，
`mujoco_sim.launch.py` 也硬编码了 `fsm_console_node` 的自动弹出逻辑，使
`adhere_mujoco_hardware` 包在编译期之外的 launch/运行时层面仍然耦合了具体控制器
与遥操作控制台包。本阶段按更新后的 `CLAUDE.md` 落地两项改动：用严格整数比例的
`MultiRateDecimator` 取代交替调度表；把 `adhere_mujoco_hardware` 相对
`adhere_kamr_controller`/`adhere_keyboard_input` 的耦合降到 0。

### 包 1：`src/controllers/adhere_controller_common`

新增 `include/adhere_controller_common/common/MultiRateDecimator.h`：纯逻辑类
（零 `rclcpp`/`hardware_interface` 依赖），构造时传入
`(cm_update_rate, inner_decimation, outer_decimation)`，`tick()` 对递增计数器做
`tick_count % inner_decimation == 0`/`tick_count % outer_decimation == 0` 的纯
取模判断并返回 `{inner_due, outer_due}`；`innerPeriodSeconds()`/
`outerPeriodSeconds()` 返回恒定的 `decimation/cm_update_rate` 秒值；`reset()`
令下次 `tick()` 的 `inner_due`/`outer_due` 同时为真。新增
`test/test_multirate_decimator.cpp`（4 个 `gtest` 用例）验证：默认
`inner_decimation=2`/`outer_decimation=20` 下 500 次 tick 精确触发 250 次内环/
25 次外环（且每次 `outer_due` 必然伴随 `inner_due`）；`outer_decimation=25`
的 `20Hz` 外环替代配置同样精确；`reset()` 后下次 tick 必为双触发。
`CMakeLists.txt`/`package.xml` 新增 `ament_cmake_gtest` 测试目标与
`test_depend`。

### 包 2：`src/controllers/adhere_kamr_controller`

- `adhere_kamr_controller.hpp`：移除按 `get_node()->now()` 差值实测 `period`
  的 `last_update_time_` 字段，改为持有
  `std::shared_ptr<adhere_controller_common::MultiRateDecimator> decimator_`、
  恒定的 `rclcpp::Duration inner_period_`、以及
  `cm_update_rate_`/`inner_decimation_`/`outer_decimation_` 三个整型成员。
- `adhere_kamr_controller.cpp`：
  - `on_configure()` 新增 `auto_declare<int>("cm_update_rate", 500)`/
    `auto_declare<int>("inner_decimation", 2)`/
    `auto_declare<int>("outer_decimation", 20)` 三个 ROS 参数读取，据此构造
    `decimator_` 并计算恒定 `inner_period_ = inner_decimation_/cm_update_rate_`
    秒。
  - `on_activate()` 新增 `decimator_->reset()`（保证每次激活都从 tick 0
    对齐重新开始）与 `ctrl_interfaces_.frequency_ = cm_update_rate_ /
    inner_decimation_`（供未来 `FSMState` 读取实际内环频率而非假设硬编码
    常量）。
  - `update()` 重写为：先 `decimator_->tick()`；`outer_due` 时执行一个当前为
    空操作的外环钩子占位（Phase 4/5 才接入真正的异步 CUDA MPPI 线程交换）；
    非 `inner_due` 的 tick 直接 `return OK`（`thruster_command_interface_`
    不触碰，天然 ZOH）；`inner_due` 的 tick 才用恒定 `inner_period_`（而非
    实测差值）调用 `current_state_->run(time, inner_period_)`，再走
    `checkChange()`/`enter()`/`exit()` 原有切换逻辑。
- 新增 `launch/mujoco.launch.py`：闭环启动文件（职责从
  `adhere_mujoco_hardware/launch/mujoco_sim.launch.py` 迁移并扩展）——启动
  `robot_state_publisher` + 通用 `adhere_mujoco_node` 可执行文件（以
  `simplified_v1_description/config/robot_control.yaml` 为 `--params-file`，
  额外叠加 `autostart_controllers:=['adhere_kamr_controller']` 这一个 ROS
  参数）+ RViz2 + 自动弹出 `fsm_console_node` 终端窗口（`spawn_fsm_console`
  默认 `true`，逻辑原样从旧 `mujoco_sim.launch.py` 迁移）。
- `package.xml` 新增 `exec_depend`：`adhere_mujoco_hardware`（供其
  `adhere_mujoco_node` 可执行文件被本包的 launch 文件引用）、
  `adhere_keyboard_input`（`fsm_console_node`）、`simplified_v1_description`
  （xacro + `robot_control.yaml`）、`xacro`/`ament_index_python`/
  `robot_state_publisher`/`rviz2`。均为单向 `exec_depend`（`adhere_mujoco_hardware`
  已不再声明任何反向依赖，不构成 colcon 构建环）。

### 包 3：`src/hardwares/adhere_mujoco_hardware`（纯化，非新增）

- `adhere_mujoco_node.cpp`：移除硬编码的
  `load_controller("adhere_kamr_controller", "adhere_kamr_controller/
  AdhereKamrController")` 二参数重载调用，改为通用的 `autostart_controllers`
  字符串数组 ROS 参数（默认空数组），对列表中每个名字循环调用单参数
  `load_controller(name)`（插件类型从本进程 `controller_manager` 参数命名空间
  解析）+ `configure_controller(name)`；末尾的 `switch_controller(...)`
  同样从硬编码的 `{"adhere_kamr_controller"}` 改为 `autostart_controllers`，
  且仅在非空时才调用。
- `launch/mujoco_sim.launch.py`：移除 `spawn_fsm_console` 参数与整段
  `fsm_console` `ExecuteProcess`（迁移至
  `adhere_kamr_controller/launch/mujoco.launch.py`）；新增通用
  `autostart_controllers`（默认 `"[]"`，经 `yaml.safe_load()` 解析，为空时
  干脆不把该键放进传给 `adhere_mujoco_node` 的参数字典——`launch_ros` 的
  `evaluate_parameter_dict()` 无法对空列表推断数组元素类型，必须整个省略
  该键，C++ 侧 `declare_parameter` 的默认值 `{}` 效果等价）。
- `package.xml` 移除 `adhere_kamr_controller`/`adhere_keyboard_input` 两条
  `exec_depend`：`grep -rn "adhere_kamr_controller\|fsm_console_node"
  src/hardwares/adhere_mujoco_hardware/` 现在对 `.cpp`/`.hpp`/`.py`/`.xml`/
  `CMakeLists.txt` 返回 0 条匹配（仅 `README.md` 保留指向
  `adhere_kamr_controller/README.md` 的文档交叉引用，非代码耦合）。

### 包 4：`src/descriptions/simplified_v1_description`

新增 `config/robot_control.yaml`：供 `adhere_kamr_controller/launch/
mujoco.launch.py` 作为单一 `--params-file` 传给 `adhere_mujoco_node`。

**关键发现（Foxy `ros2 launch` 的 `__node` 重映射陷阱）**：`launch_ros` 的
`Node` 动作总会附加一条无作用域的 `-r __node:=<name>` 重映射规则（`<name>`
即该 launch 动作赋予可执行文件的名字，此处是 `adhere_mujoco_node`）。这条
规则对进程内**所有**使用默认 `NodeOptions` 构造的节点生效——不只是顶层
`rclcpp::Node("adhere_mujoco_node")`，连 C++ 里显式命名为
`"controller_manager"` 的内部 `ControllerManager` 节点、以及
`AdhereKamrController` 自己的 `controller_interface` 节点，三者在 ROS 图里
全部显示为同一个 `/adhere_mujoco_node`（用 `ros2 node list` 实测确认，
WARNING 提示三个节点同名）。最初按直觉把 `controller_manager`/
`adhere_kamr_controller` 的 `ros__parameters` 分别写成两个顶层键
`controller_manager:`/`adhere_kamr_controller:`，导致
`load_controller(name)` 报错 `"The 'type' param not defined for
'adhere_kamr_controller'"`——用一个临时 `RCLCPP_INFO` 诊断日志打印
`get_node()->get_name()`/`get_fully_qualified_name()` 并把
`outer_decimation` 临时改成哨兵值（`7`、`11`）两轮经验验证后，确认
`AdhereKamrController` 自己的节点也被重映射为 `/adhere_mujoco_node`，于是
把两个顶层键合并为同一个 `adhere_mujoco_node:` 键（`controller_manager` 的
`update_rate`/`<name>.type` 嵌套参数与 `AdhereKamrController` 自己的
`cm_update_rate`/`inner_decimation`/`outer_decimation`/`kp_z`/`ki_z`/`kd_z`
平铺参数共存于同一 `ros__parameters` 块）后，诊断日志确认哨兵值能正确传递，
问题解决。验证完毕后移除诊断日志，`outer_decimation` 改回正式值 `20`。
此发现已记录到 `adhere_kamr_controller/README.md`。

### 验证结果

`colcon build --symlink-install`（全部 6 个包）**零警告零错误**。
`colcon test --packages-select adhere_controller_common adhere_mujoco_hardware`：
新增的 `test_multirate_decimator`（4/4）与既有的
`test_fossen_hydrodynamics_geometry`（26/26）、
`test_rhombic_kern_and_thruster_model`（9/9）**全部通过**，对本阶段改动零
回归（既有 `copyright`/`cpplint`/`flake8`/`pep257`/`uncrustify` style-lint
失败是 Phase 2.14 起已记录的"全仓库缺版权头"预先存在问题，与本阶段无关，
已用 `git stash` 对照确认新增文件未引入任何新的 lint 失败行）。

`ros2 launch adhere_mujoco_hardware mujoco_sim.launch.py headless:=true
use_rviz:=false` 确认纯开环：`list_controllers` 服务返回空数组（0 个控制器），
`/rov/odom` 正常以被动漂流发布。`ros2 launch adhere_kamr_controller
mujoco.launch.py headless:=true use_rviz:=false spawn_fsm_console:=false`
确认完整闭环（经 `ros2 topic pub /rov/fsm_command` 脚本化驱动，默认在系统上
已有一个独立运行的 `fsm_console_node`，故测试时关闭自动弹出）：

| 阶段 | 目标 | 终值 `z` | 残差 |
|---|---|---|---|
| `PASSIVE`（默认启动） | 自由漂流 | `1.0030m` | — |
| `DEPTH_HOLD` | `z_des=0.40m` | `0.39998m` | `0.018mm` ✅ `<5mm` |
| `DEPTH_HOLD`（运行时重定目标） | `z_des=0.75m` | `0.75061m` | `0.61mm` ✅ `<5mm` |
| `PASSIVE`（切回） | 自由漂流 | `1.0030m`（恢复原平衡位置） | — |

`list_controllers` 服务在闭环启动后返回 `adhere_kamr_controller`
（`state='active'`，`type='adhere_kamr_controller/AdhereKamrController'`，
claimed 全部 8 个 `thr_*/effort` 命令接口）。

## Phase 3.2（2026-10-17）：4 话题解耦通信契约、共享 `ThrusterAllocator`、`OPEN_LOOP`/`PID_HOLD` 两态、官方 `joy` 包 Xbox 手柄遥操作、上位机（GCS）独立部署

### 动机

Phase 3.1 把 FSM 状态切换与状态内设定点（定深目标）揉进同一个 `FsmCommand.msg`
与同一个单体终端循环，难以扩展到更多控制模式与硬件遥操作。本阶段落地
`CLAUDE.md` Phase 3.2 的 4 话题正交解耦契约，新增 `StateOpenLoop`/`StatePidHold`
两个 FSM 状态用 Xbox 手柄验证 `B`/`B^+` 分配与闭环动力学补偿，并让
`adhere_control_msgs`/`adhere_keyboard_input` 可独立部署到不装 MuJoCo/CUDA/
`ros2_control` 的远程上位机（GCS）笔记本。

**对任务文本的一处偏离（已与用户确认）**：任务描述中 `ThrusterAllocator` 的
几何（`thr_N1..4` 在 `X=-0.10m` 杠杆臂、`thr_T5/T6` 用 `gear="0 1 0 0 0 0"` 等）
与真实 `rov_wall.xml` 不符（该文件里全部 8 个推力器站点都在 `X=0.0`，与质心
同一 X 坐标，`thr_T5..T8` 是 4 个在 Y-Z 平面内 30° 矢量角的切向推力器）。这正是
`phase2_status_and_decisions` 记忆中已记录的 `CLAUDE.md` 占位几何与真实 MJCF
不符问题。按用户决定，`ThrusterAllocator` 复用 `StateDepthHold.cpp` 已有的、
基于真实几何推导的 `buildAllocationMatrix()`，而非任务文本的占位几何。

### 包 1：`src/interfaces/adhere_control_msgs`

重写 `FsmCommand.msg`（去掉 `target_z/target_y/target_yaw`，新增
`OPEN_LOOP=3, PID_HOLD=4` 常量，只保留 `command` 一个字段）；新增
`SetpointCommand.msg`（离散设定点）、`TeleopCommand.msg`（50Hz 连续遥操作）、
`FsmState.msg`（控制器实时反馈）。`CMakeLists.txt` 的
`rosidl_generate_interfaces()` 列出全部 4 个 `.msg`。

### 包 2：`src/controllers/adhere_controller_common`

- `common/enumClass.h`：`FSMStateName` 新增 `OPEN_LOOP`/`PID_HOLD`。
- 新增 `common/ThrusterAllocator.h`（header-only）：把 `StateDepthHold.cpp` 原有
  的 `buildAllocationMatrix()` 几何搬到这里作 `buildThrusterAllocationMatrix()`
  共享实现，新增 `buildThrusterAllocationPseudoInverse()` 实现任务指定的阻尼
  Moore-Penrose 伪逆 `B^T(BB^T + 1e-6*I_6)^-1`（`rank(B)=6`，阻尼项数值上可忽略，
  仅做条件数安全边际）。`StateDepthHold`/`StateOpenLoop`/`StatePidHold` 三者
  统一调用这一对函数，不再各自维护几何推导副本。
- `CtrlInterfaces.h` 的 `ControlInputs` 新增 `setpoint_*`/`setpoint_seq`
  （`SetpointCommand` 镜像 + 沿边检测序号）与 `teleop_*`/`teleop_last_rx_time_sec`
  （`TeleopCommand` 镜像 + 看门狗时间戳）两组原子字段。`FSMState` 基类新增
  非虚共享方法 `checkFsmCommandChange()`（把 `command` 映射到目标
  `FSMStateName`，4 个具体状态的 `checkChange()` 均只需转发它，取代 Phase 3.1
  每个状态各自硬编码 `== 1`/`== 2` 的写法）与虚方法 `targetPose()`（默认全零，
  供 `/rov/fsm_state::target_pose` 使用）。
- `StateDepthHold` 新增真正的 `target_z_` 成员（Phase 3.1 版本直接读
  `control_inputs_.target_z` 原子量，没有本地状态）：`enter()` 用当前世界 Z
  做种；`run()` 先按 `setpoint_seq` 沿边应用 `SetpointCommand`（`mode==2`），
  再无条件叠加 Xbox 左摇杆上下的连续微调
  `target_z_ += 0.30 * teleop.heave_z * dt`（摇杆回中后天然锁定，无需额外
  "release" 分支），最终钳制到 `[0.10, 0.95]m`。
- 新增 `StateOpenLoop`：无记忆地把 `/rov/teleop_cmd` 的 4 维输入映射为机体力矩
  （`Fx=-60*surge_x, Fy=-50*sway_y, Fz=50*heave_z, Tz=8*yaw_rate`），经 `B^+`
  分配并钳制 `±40N`。
- 新增 `StatePidHold`：`enter()` 锁存当前 `(x,y,z,yaw)`；`run()` 中
  `teleop.active` 为真时按机体对齐速度平移/旋转设定点（钳制 `x_d>=0.12m`，
  `z_d∈[0.10,0.95]`），为假时速度目标与设定点均冻结；6-DOF 机体帧 PID（新调的
  `Phase 3.2` 增益，`Z` 轴复用 `StateDepthHold` 已验证的 `180/95/62`）+
  浮力/科氏力阻力前馈（浮力常数直接抄 `fossen_hydrodynamics.hpp` 的
  `HydroParams` 默认值，而非反向依赖 `adhere_mujoco_hardware`，保持
  `CLAUDE.md` §3 的包解耦约束；净浮力用**干重** `20kg`，不是 MJCF `<inertial>`
  的 26kg 增广质量——增广质量是水动力附加质量的惯性反应项，不是真实重力，
  得到的净浮力 `rho*g*V - m_dry*g = 198.162 - 196.2 = +1.962N` 与任务给出的数值
  一致）。
- 新增 `test/test_thruster_allocator.cpp`（3 个 gtest：`B*B^+ ~= I_6` 在
  `1e-4` 内；纯 `Fy`/`Fz` 指令零寄生力矩）与 `test/test_fsm_states.cpp`（3 个
  闭环 gtest：用 `hardware_interface::StateInterface/CommandInterface` 的
  `(name, interface_name, double*)` 构造函数搭一个假 `CtrlInterfaces`——无需
  `ResourceManager`——配合测试自身充当的简单质点欧拉积分"虚拟物理"，验证
  `StateDepthHold` 对 `SetpointCommand`+摇杆微调的 `<5mm` 收敛、`StateOpenLoop`
  零寄生力矩、`StatePidHold` 的移动+释放锁定 `<5mm`）。`CMakeLists.txt` 新增
  对应的 `ament_add_gtest` 目标。

### 包 3：`src/controllers/adhere_kamr_controller`

`on_configure()` 订阅改为 3 个输入话题（`/rov/fsm_command` 只写
`control_inputs_.command`；`/rov/setpoint_cmd` 写 `setpoint_*` 并把
`setpoint_seq` 自增；`/rov/teleop_cmd` 写 `teleop_*` 并戳
`teleop_last_rx_time_sec`），新增 `/rov/fsm_state` 发布者。`FSMStateList`/
`getNextState()` 新增 `open_loop`/`pid_hold`。`update()` 的内环 tick 上先做
`0.25s` 失联看门狗（陈旧则把 `teleop_active` 与 4 个遥操作轴集中清零，各
`FSMState` 自身不用关心断联处理），外环 tick 上发布 `/rov/fsm_state`
（`target_pose` 取自 `current_state_->targetPose()`，`actual_pose` 由
`odom_state_interface_` 直接算出 `[x,y,z,yaw]`）。
`launch/mujoco.launch.py` 把原来硬编码的 `spawn_fsm_console` + 独立
`ExecuteProcess` 换成 `launch_teleop`（默认 `true`）+
`IncludeLaunchDescription(adhere_keyboard_input/launch/teleop_gcs.launch.py)`。

### 包 4：`src/commands/adhere_keyboard_input`

- `fsm_console_node` 重写：订阅 `/rov/fsm_state`（不再订阅 `/rov/odom`），只发布
  `FsmCommand{command}`；`current_state` 变化时用
  `fork()+setsid()+execl()` 动态弹出/`killpg()` 清理
  `state_cmd_console_node`（`gnome-terminal --wait`/`xterm` 回退，与
  `mujoco.launch.py` 既有弹窗惯用法一致，`--wait`/`setsid()` 保证 fork 出的
  pid 真能代表并杀掉整条终端+内部 `ros2 run` 进程链）。
- 新增 `state_cmd_console_node`（ROS 参数 `mode=2|4`）：订阅 `/rov/fsm_state`
  显示 Actual/Target，只发布 `SetpointCommand`；`mode=4` 下 `z`/`y`/`x`/`yaw`
  单轴部分更新会把当前已知 `target_pose` 回填到另外 3 轴再覆盖被改的那一个。
- 新增 `xbox_teleop_node`：订阅官方 `joy` 包的 `/joy`，在回调里同步发布
  `/rov/teleop_cmd`（死区 `0.08` + 三次方曲线，默认轴映射
  `axis_heave=1,axis_yaw=0,axis_surge=4,axis_sway=3`，匹配 Linux `xpad` 默认
  8 轴布局）与按键上升沿去抖触发的 `/rov/fsm_command`（默认 `A/B/X/Y ->
  PASSIVE/PID_HOLD/DEPTH_HOLD/OPEN_LOOP`）。新增 `config/xbox_teleop.yaml`。
- 新增 `launch/teleop_gcs.launch.py`（`joy_node` + `xbox_teleop_node` +
  `fsm_console_node`，零 MuJoCo/CUDA/`ros2_control` 依赖）。
- `package.xml`/`CMakeLists.txt`：去掉未再使用的 `nav_msgs`/`geometry_msgs`，
  新增 `sensor_msgs`（`depend`）与 `joy`（`exec_depend`，运行期依赖，本包不
  include 任何 `joy` 头文件）。

### 验证结果

`colcon build --symlink-install`（全部 6 个包）**零警告零错误**。
`colcon test --packages-select adhere_controller_common`：新增的
`test_thruster_allocator`（3/3）、`test_fsm_states`（3/3）与既有的
`test_multirate_decimator`（4/4）**全部通过**；`cpplint` 对新文件报出的
`line_length`/缺 `#include <algorithm>` 问题已修复，`copyright`/`cppcheck`/
`uncrustify`/`xmllint` 失败均为 Phase 2.14 起已记录的"全仓库缺版权头"等预先
存在问题，与本阶段无关。

真实 MuJoCo 闭环（`ros2 launch adhere_kamr_controller mujoco.launch.py
headless:=true use_rviz:=false launch_teleop:=false`，经 `rclpy` 脚本驱动 4
个 `/rov/*` 话题）：

| 状态 | 驱动 | 结果 |
|---|---|---|
| `PASSIVE`（默认启动） | — | `z=1.0029m`（自由漂流，与 Phase 3.1.1 基线一致） |
| `DEPTH_HOLD` | `SetpointCommand{target_z=0.40}` | `z=0.399999995m`，残差 `<0.01mm` ✅ |
| `OPEN_LOOP` | `TeleopCommand{sway_y=1.0}` 持续 2s | `Y` 平移约 `-8.4m`，`Z`/姿态几乎不变（`z=1.0031m`），确认零寄生俯仰/偏航耦合 ✅ |
| `PID_HOLD`（驱动中） | `TeleopCommand{surge_x=-1.0}` 持续 3s | `target_pose.x` 持续前移，`actual` 跟随（有追踪滞后，符合"移动中"预期） |
| `PID_HOLD`（释放后） | 摇杆回中，再等 8s | `actual` 收敛到释放瞬间的 `target`，`X/Y/Z` 残差 `<1e-6m` ✅ 远优于 `<5mm` 契约 |

另用合成 `sensor_msgs/msg/Joy` 消息独立验证 `xbox_teleop_node`（未接真实手柄）：
死区`0.08`+三次方曲线对 `axis=0.5`/`-0.9` 的变换结果与手算一致；`buttons[2]`
（X 键）按住两帧只触发一次 `FsmCommand{command=DEPTH_HOLD}`，确认上升沿去抖
生效。

## Phase 3.3（2026-10-17）：`adhere_controller` 包合并、模型参数完全解耦、水箱坐标系重定义（水面 `Z=0`）

### 动机

Phase 3.1~3.2 把控制器拆成 `adhere_controller_common`（基础设施）与
`adhere_kamr_controller`（算法主体+`pluginlib` 插件）两个包，但二者从未独立
编译/部署过，纯粹的包拆分只带来跨包链接与维护开销；同时推力器几何、贴垫坐标、
Fossen 水动力参数、各 FSM 状态 PID 增益仍是 `StateDepthHold.cpp`/
`StateOpenLoop.cpp`/`StatePidHold.cpp`/`ThrusterAllocator.h` 里的 `constexpr`
常量，换模型需要改 C++ 并重新编译；水箱坐标系仍是旧的水面 `Z=+1.0m`/
`4m x 4m` 壁面约定，与 `CLAUDE.md` 最新改写版本（水面 `Z=0.0m`、箱底
`Z=-10.0m`、`20m x 10m` 壁面）不符。本阶段落地三项改动并把结果同步进
`CLAUDE.md` 新增的第 9 节"当前代码库快照"。

### 包合并：`src/controllers/adhere_controller`

`git mv adhere_controller_common adhere_controller`，把 `adhere_kamr_controller`
的 `adhere_kamr_controller.cpp/.hpp`、`adhere_kamr_controller.xml`、
`launch/mujoco.launch.py`、`config/controllers.yaml` 迁入同一个包，命名空间
`adhere_controller_common`/`adhere_kamr_controller` 全局改写为单一
`adhere_controller`，类 `AdhereKamrController` 改名 `AdhereController`，插件
字符串改为 `adhere_controller/AdhereController`；`git rm -rf` 旧
`adhere_kamr_controller` 目录。合并后的 `CMakeLists.txt`/`package.xml` 把
`pluginlib`/`controller_interface` 导出与 3 个既有 gtest 目标
（`test_multirate_decimator`/`test_thruster_allocator`/`test_fsm_states`）
全部搬进单一包，不再有包间 `depend`。`adhere_mujoco_hardware` 对旧包名的
依赖此前已为 0（Phase 3.1.1 解耦的遗留验证），本阶段仅确认无需改动。

### 模型解耦：`ModelParams` + 动态 `ThrusterAllocator`

新增 `include/adhere_controller/common/ModelParams.h`：纯数据结构体，持有
推力器几何（`thruster_pos`/`thruster_dir`/`thruster_min`/`thruster_max`，
`std::array<Vec3,8>`/`std::array<double,8>`，默认值即 `rov_wall.xml` 的真实
几何）、贴垫几何（`pad_pos`，供未来 WBC）、Fossen 水动力前馈参数
（`rho_water/gravity/vehicle_mass/augmented_mass/displaced_volume/
cob_offset_body/z_surface/z_transition/D_lin/D_quad/D_ang_lin/D_ang_quad`）、
工作空间安全边界（`workspace_z_min/max/x_min`）与三个状态增益子结构体
（`DepthHoldGains`/`OpenLoopGains`/`PidHoldGains`）。静态方法
`loadFromNode(rclcpp::Node::SharedPtr)` 把每个字段声明为 ROS 参数并读回，
在 `AdhereController::on_configure()` 中调用一次，构造出的 `model_params_`
以 `const ModelParams&` 传给全部 4 个 `FSMState` 构造函数。`ThrusterAllocator.h`
的 `buildThrusterAllocationMatrix()` 从硬编码的角点/`zaxis` 几何改写为通用的
叉乘公式 `b_i = [dir_i; pos_i x dir_i]`，接受 `ModelParams` 提供的任意几何
数组，对当前 `rov_wall.xml` 几何代数等价（`test_thruster_allocator.cpp` 的
`B*B^+ ~= I_6` 断言保持不变地验证了这一点）。`StateDepthHold`/`StateOpenLoop`/
`StatePidHold` 内部全部 `constexpr`/`const Vec3` 增益与边界常量替换为
`model_params_.*` 字段读取；`StatePidHold` 的 buoyancy/drag 前馈改用
`model_params_` 里的水动力参数而非本地硬编码副本（数值与之前完全一致，仅来源
变为运行时参数）。`PidHoldGains` 采用 `CLAUDE.md` 第 2 节 Stage 2.4 给出的
全新增益（`kp_pos=[120,120,120]`、`kp_att=[40,40,25]` 等，比 Phase 3.2
的调试值更硬），闭环验证（见下）确认收敛性不受影响。

`simplified_v1_description/config/robot_control.yaml` 按 `CLAUDE.md` 第 2 节
Stage 2 populate 全部新键，沿用既有的单顶层键 `adhere_mujoco_node:` 约定
（`controller_manager`/`AdhereController` 自身节点均被 `ros2 launch` 的
`-r __node:=` 重映射到同一个节点名，已在 Phase 3.1.1 记录）。`thruster_pos_xyz`/
`thruster_dir_xyz` 沿用 Phase 3.2 已确认的偏离决定——编码 `rov_wall.xml` 的
真实几何（8 个推力器站点全部在 body `X=0.0`，与质心同一 `X` 坐标，零杠杆臂），
而非 `CLAUDE.md` 第 4.3 节的示意性 `X=-0.10m` 占位几何。

**关键发现（`automatically_declare_parameters_from_overrides` 与
`ParameterAlreadyDeclaredException`）**：`ModelParams::loadFromNode()` 最初
直接调用 `node->declare_parameter<T>(name, default)`，在真实 `ros2 launch`
闭环下 `adhere_mujoco_node-2` 进程在 `on_configure()` 内对 `thruster_pos_xyz`
抛出 `rclcpp::exceptions::ParameterAlreadyDeclaredException` 并 `terminate`。
原因：`controller_interface::ControllerInterfaceBase::auto_declare<T>()`
（既有 `cm_update_rate`/`inner_decimation`/`outer_decimation` 三个参数读取
用的就是这个模板方法）内部先 `has_parameter()` 判空才决定
`declare_parameter()` 还是 `get_parameter()`——因为 `controller_manager`
构造控制器自己的节点时启用了 `automatically_declare_parameters_from_overrides`，
`robot_control.yaml` 里出现的每一个键在 `on_configure()` 运行之前就已经被
自动声明好了。`ModelParams::loadFromNode()` 补上同样的 `has_parameter()`
判空封装（`autoDeclareVec`/`autoDeclareDouble` 两个本地 lambda）后问题解决。
此发现补充进 `ModelParams.h` 的行内注释。

### 水箱坐标系重定义（水面 `Z=0.0m`，箱底 `Z=-10.0m`，壁面 `20m x 10m`）

`simplified_v1_description/mjcf/rov_wall.xml`：`water_surface` 平面
`pos="0 0 0"` `size="10.0 10.0 0.01"`；`floor_geom`（箱底）`pos="0 0 -10.0"`
`size="10.0 10.0 0.05"`；`wall` 刚体 `pos="-0.01 0 -5.0"`，`wall_geom`
`size="0.01 10.0 5.0"`，`water_boundary_marker` 局部 `pos="0.02 0 5.0"`
（刚体整体下移后仍标记世界 `Z=0`）；`rov_base` 出生点改为
`pos="0.125 0 -2.0"`。贴垫/推力器/惯量等体坐标系下的几何量（相对质心的偏移）
无需改动。`adhere_mujoco_hardware/fossen_hydrodynamics.hpp` 的
`HydroParams::z_surface` 默认值 `1.0 -> 0.0`；`mujoco_viewer.cpp` 的 4 个相机
预设（`resetCameraView`/`setCameraWallYZ`/`setCameraTopXY`/`setCameraSideXZ`）
的 `cam_.lookat[2]` 全部改为 `-2.0`；`adhere_mujoco_node.cpp` 的
`/rov/environment_markers` 壁面 Marker 改为 `position={-0.01,0,-5.0}`、
`scale={0.02,20.0,10.0}`，新增水面 Marker（`Z=0`，`scale=[20,20,0.01]`，青色
半透明）与箱底 Marker（`Z=-10`，`scale=[20,20,0.05]`）。

`test_fossen_hydrodynamics_geometry.cpp`：全文件 25 处依赖"默认 `z_surface`
下处于完全浸没态"语义的 `com_pos_world`/`qpos[2]` 测值从旧约定下的 `0.5`
统一替换为新约定下的 `-2.0`（`submerged_ratio` 公式
`clamp((z_surface+z_transition-z)/z_transition,0,1)` 在两种取值下都恰好
饱和于 `1.0`，故除注释外全部既有数值断言逐位不变）；`sphere_body`/
`spheroid_body` 两个纯几何/附加质量测试（与 `z_surface` 无关）按计划保持
`pos="0 0 0.5"` 不变。

`adhere_keyboard_input/state_cmd_console_node.cpp`：新增
`depthInputToTargetZ()`，`mode==2`（及 `mode==4` 的 `z <val>`/`all` 分支）
统一把输入值归一化为 `target_z = -|input|`，钳制到
`[workspace_z_min, workspace_z_max] = [-9.80, -0.15]`——用户输入 `2.5` 或
`-2.5` 得到同一个 `target_z=-2.5`；终端提示改为同时显示
`Current/Target Depth` 与对应的原始 `Z` 值。

### 验证结果

`rm -rf build/{adhere_controller_common,adhere_kamr_controller}
install/{adhere_controller_common,adhere_kamr_controller}` 后
`colcon build --symlink-install`（全部 5 个包：`adhere_control_msgs`、
`simplified_v1_description`、`adhere_mujoco_hardware`、`adhere_controller`、
`adhere_keyboard_input`）**零警告零错误**。`colcon test --packages-select
adhere_controller adhere_mujoco_hardware`：`adhere_controller` 的
`test_multirate_decimator`（4/4）、`test_thruster_allocator`（3/3，已改为从
`ModelParams` 默认几何构造）、`test_fsm_states`（3/3，已改为
`ModelParams`-based 构造函数，`DEPTH_HOLD`/`PID_HOLD` 测值改到新 `Z<0` 约定
且保持 `<5mm` 契约）与 `adhere_mujoco_hardware` 的
`test_fossen_hydrodynamics_geometry`（26/26）、
`test_rhombic_kern_and_thruster_model`（9/9）**全部通过**（既有
`copyright`/`cpplint`/`flake8`/`pep257`/`uncrustify` style-lint 失败是
Phase 2.14 起已记录的"全仓库缺版权头"预先存在问题，与本阶段无关）。

`ros2 launch adhere_controller mujoco.launch.py headless:=true use_rviz:=false
launch_teleop:=false` 真实 MuJoCo 闭环，经 `rclpy` 脚本驱动 4 个 `/rov/*` 话题
（`list_controllers` 确认 `adhere_controller`/`AdhereController` 正确加载/
激活，claimed 全部 8 个 `thr_*/effort` 命令接口）：

| 阶段 | 驱动 | 结果 |
|---|---|---|
| `DEPTH_HOLD` | `SetpointCommand{target_z=-3.5}` | `z=-3.50000`，残差 `<0.001mm` ✅ |
| `DEPTH_HOLD`（运行时重定目标） | `SetpointCommand{target_z=-1.2}` | `z=-1.20000`，残差 `<0.001mm` ✅ |
| `PID_HOLD`（驱动中） | `TeleopCommand{surge_x=-1.0}` 持续 5s | `target_pose.x` 从 `0.125` 前移至 `2.135` |
| `PID_HOLD`（释放后） | 摇杆回中，settle 25s | `actual_x` 收敛到释放瞬间 `target_x`，残差 `0.056mm`；`y`/`z` 残差同样 `<0.001mm` ✅ 均远优于 `<5mm` 契约 |

（注：headless 模式下物理步进速率明显快于/慢于挂钟实时，脚本化验证统一预留
`≥25s` 挂钟settle 窗口，而非直接照搬 Phase 3.2 单元测试里 `4ms*4000=16s`
仿真步数的挂钟估算——首次用 `5s/8s` 窗口验证时 `DEPTH_HOLD`/`PID_HOLD` 均显示
"仍在收敛但未到 `<5mm`"，加长到 `25s` 后确认二者均已早早饱和收敛，并非增益或
几何改动引入的回归。）

## Phase 3.4（2026-10-17）：启动文件迁移至 `simplified_v1_description`，实现真正的模型-描述耦合

### 动机

`mujoco_sim.launch.py`（纯开环，0 控制器）此前一直放在 `adhere_mujoco_hardware`
包内，`mujoco.launch.py`（闭环，自动加载 `adhere_controller`）放在
`adhere_controller` 包内——二者都是"工具包反向持有模型/场景启动逻辑"的架构
违例：`CLAUDE.md` 的核心解耦原则要求 Plant/Model（`simplified_v1_description`）
与 Tools/Controllers（`adhere_mujoco_hardware`/`adhere_controller`）严格分离，
换一个新 ROV 模型应该只需要运行该模型自己描述包下的同名启动文件，而不是去改
两个完全通用的工具包。

### 迁移与合并

把两个旧启动文件的全部能力合并进单一的
`src/descriptions/simplified_v1_description/launch/mujoco.launch.py`：
`headless`/`use_rviz`/`pkg_description`/`mjcf_file`/`xacro_file`（原
`mujoco_sim.launch.py` 的跨模型零重编参数）+ `robot_control_yaml`/
`autostart_controllers`（原 `adhere_controller/launch/mujoco.launch.py` 的
"generic autostart_controllers in-process 加载机制"，默认改为
`['adhere_controller']` 以匹配 README 里"完整闭环仿真"的默认用法描述）+
`launch_teleop`（含 `adhere_keyboard_input/teleop_gcs.launch.py` 的 joy_node +
xbox_teleop_node + fsm_console_node 整束）。原纯开环行为（0 控制器）通过
`autostart_controllers:="[]" launch_teleop:=false` 覆盖参数即可还原，不是单独
的启动文件。`git rm -rf` 删除 `adhere_mujoco_hardware/launch/` 与
`adhere_controller/launch/` 两个目录（`CMakeLists.txt` 对应的
`install(DIRECTORY launch ...)` 行一并移除），二者此后不再持有任何启动文件。

`package.xml`：`adhere_mujoco_hardware`/`adhere_controller` 里原本只为自己的
启动文件声明的 `exec_depend`（`xacro`/`ament_index_python`/`rviz2`/
`robot_state_publisher`，`adhere_controller` 还额外有
`adhere_mujoco_hardware`/`adhere_keyboard_input`/`simplified_v1_description`）
全部移除；`simplified_v1_description/package.xml` 沿用 Phase 2.12 就已确立的
"运行时按包名引用、不声明 `exec_depend`，避免 colcon 构建顺序环"约定（与对
`adhere_mujoco_hardware` 的既有处理方式一致），不新增任何依赖声明。RViz 配置
文件沿用 `adhere_mujoco_hardware/config/mujoco_sim.rviz`（不移动）。

### 验证结果

`rm -rf build/{adhere_mujoco_hardware,adhere_controller,simplified_v1_description}
install/{adhere_mujoco_hardware,adhere_controller,simplified_v1_description} log`
后 `colcon build --symlink-install`（全部 5 个包）**零警告零错误**；确认
`install/` 下仅剩 `simplified_v1_description/share/simplified_v1_description/
launch/mujoco.launch.py` 一份启动文件，`adhere_mujoco_hardware`/
`adhere_controller` 的 `share/.../launch/` 均不存在。`colcon test
--packages-select adhere_controller adhere_mujoco_hardware`：全部既有 gtest
（`test_multirate_decimator`/`test_thruster_allocator`/`test_fsm_states`/
`test_fossen_hydrodynamics_geometry`/`test_rhombic_kern_and_thruster_model`）
零失败；仅剩 Phase 2.14 起已记录的"全仓库缺版权头"等预先存在的
`copyright`/`cpplint`/`cppcheck`/`uncrustify` style-lint 失败，与本阶段无关。

`ros2 launch simplified_v1_description mujoco.launch.py headless:=true
use_rviz:=false launch_teleop:=false`（默认 `autostart_controllers`）真实
启动：`robot_state_publisher` + `adhere_mujoco_node` 正常加载/配置/激活
`adhere_controller`/`AdhereController`（`ros2 control list_controllers
--controller-manager /adhere_mujoco_node` 显示 `active`），全部 4 个
`/rov/*` 话题存在。追加 `autostart_controllers:="[]"` 覆盖后同一条命令同样
无错误启动且 0 个控制器加载，确认两种历史行为都可从这一份启动文件复现。

## Phase 3.5（2026-10-18）：双重状态获取模式（`ground_truth` / `sensor_fusion`）与 Xbox 扳机俯仰/横滚遥操作

### 动机

此前 `AdhereMujocoHardware` 只暴露无噪声地面真值（`imu_sensor`/`rov_odom`/
`pad_contact`），`CtrlInterfaces` 把这份地面真值原样交给每一个 `FSMState`。
真实 Jetson 部署时不存在地面真值——只有安装在不同机体位置、采样率各异、带
噪声的 IMU/深度计/DVL/接触垫载荷传感器。本阶段新增与地面真值完全独立的第二
条噪声多速率传感路径，新增 `RovStateEstimator` 500Hz 融合，并**严格禁止**
地面真值在 `sensor_fusion` 模式下进入任何 `FSMState::run()`；同时把 Xbox
遥操作从 4-DOF 补全为 6-DOF（左扳机 LT/LB 控制俯仰角速度，右扳机 RT/RB
控制横滚角速度）。

### `adhere_control_msgs`：消息 4 话题契约扩展为 6-DOF

`TeleopCommand.msg` 新增 `float64 pitch_rate`/`float64 roll_rate`；
`SetpointCommand.msg` 新增 `float64 target_roll`/`float64 target_pitch`；
`FsmState.msg` 把原 4 维 `target_pose`/`actual_pose` 重做为 `string
estimation_mode` + `float64[6] target_pose` + `float64[6] estimated_pose`
+ `float64[6] ground_truth_pose`（均为 `[x,y,z,roll,pitch,yaw]`）——
`estimated_pose` 是控制器实际使用的状态（地面真值或融合估计，取决于
`estimation_mode`），`ground_truth_pose` 仅供离线评估参考，从不反馈进控制环。

### `simplified_v1_description`：传感器安装位 + 启动参数

`mjcf/rov_wall.xml` 新增 7 个 `<site>`：`imu_site`/`depth_site`（质心
`(0,0,0)`）、`dvl_site`（机体底面 `(0,0,-0.20)`）、`loadcell_TL/TR/BL/BR`
（与贴垫同坐标 `(-0.10,±0.25,±0.15)`）；`xacro/robot.xacro` 新增对应的
`imu_link`/`depth_link`/`dvl_link`/`loadcell_*_link` 静态 TF 标记（纯可视化，
无物理）。`config/robot_control.yaml` 新增 `sensor_mount.r_imu_xyz/
r_depth_xyz/r_dvl_xyz`（供 `RovStateEstimator` 的杠杆臂补偿公式使用）、
`pid_hold.max_teleop_vel` 由 4 元扩展为 6 元（含俯仰/横滚速率上限）、
`pid_hold.max_tilt_deg: 45.0`。**关键发现**：`estimation_mode` 最初也被写进
`robot_control.yaml`（顶层键 `adhere_mujoco_node:`），导致 `mujoco.launch.py`
的 `estimation_mode:=sensor_fusion` 启动参数被静默忽略——ROS 2
参数合并规则是"精确节点名匹配优先于通配符 `/**:` 匹配，且与命令行顺序无关"，
而 `ros2 launch` 的 `Node` 动作把 Python dict 形式的覆盖参数编译成通配符
`/**:` 临时 YAML，天然在精确匹配面前总是输（无论列表顺序）。修复方式：把
`estimation_mode` 从 `robot_control.yaml` 中整条删除（只保留在 YAML 里写明
"为什么不能写在这里"的说明注释），改为完全依赖
`mujoco.launch.py` 新增的 `estimation_mode` 启动参数（默认
`"ground_truth"`）与 `ModelParams` 的编译期默认值。该发现同时解释了为什么
`headless`/`pkg_description`/`mjcf_file` 这几个既有的 launch 覆盖参数从未
触发过同样的 bug——它们从未在 `robot_control.yaml` 里重复声明过同名键。

### `adhere_mujoco_hardware`：独立的噪声多速率传感路径

`hw_states_` 从 36 维扩容到 55 维：新增 `kImuNoisy`（10：四元数+陀螺+加计）、
`kDepthNoisy`（1）、`kDvlNoisy`（4：速度 xyz + 声学测距）、`kLoadcellNoisy`
（4）。`configure()` 新增 `imu_site_id_`/`depth_site_id_`/`dvl_site_id_`/
`loadcell_site_id_[4]` 解析（与既有 `pad_yz_` 解析完全同构，从不硬编码）。
`read()` 新增：IMU 陀螺=真值角速度+随机游走偏置+高斯噪声（`gyro_bias_rw_std`
每 tick 游走一次）、加计=既有有限差分线加速度（含杠杆臂离心/切向项，当前模型
`r_imu=0` 故恒为零，但公式保持通用）+噪声、姿态=真值四元数叠加小角度高斯
扰动四元数；深度计/DVL 按 `step_count_ % decimation == 0`（`50Hz`/`20Hz`）
采样保持（ZOH，直接复用 `hw_states_` 自身做保持存储，无需额外成员）；
载荷传感器=既有真值贴垫力+高斯噪声，钳制 `max(0, ·)`。噪声标准差/偏置游走
标准差/采样保持周期全部硬编码在 `adhere_mujoco_hardware.cpp` 匿名命名空间常量
里（镜像 `HydroParams` 的"编译期默认值 + 可选 MJCF 覆盖"先例的文档注释，本阶段
未实现覆盖机制），因为这个硬件插件本身不是 `rclcpp::Node`，没有 ROS 参数
读取能力。`xacro/ros2_control.xacro` 新增 `imu_sensor_noisy`/
`depth_sensor_noisy`/`dvl_sensor_noisy`/`loadcell_noisy` 4 个 `<sensor>`
声明（8 command + 55 state interface）。**调试记录**：该文件顶部注释里用
`--` 分隔小标题（如 `"... ground truth"`），触发了 XML 规范对注释内容禁止
出现字面 `--` 的硬性限制（`xml.dom.minidom`/`xacro` 解析报
`not well-formed (invalid token)`），改用 `,` 分隔后解决——纯 XML/MJCF 文件的
注释与 YAML `#` 注释的转义规则不同，不能混用习惯写法。

### `adhere_controller`：`RovStateEstimator` + 严格地面真值隔离

新增 `include/adhere_controller/estimator/RovStateEstimator.h`（纯数学头文件，
无 ROS/MuJoCo 依赖，风格与 `MultiRateDecimator.h` 一致）：持有
`p_hat_world_/v_hat_body_/omega_hat_body_/q_hat_` 四元数姿态表示；
`predict()` 用噪声陀螺积分四元数+噪声加计死推算机体速度/世界位置；
`correctAttitude()` 每 tick 用噪声 IMU 姿态四元数做符号对齐的 nlerp 互补修正；
`correctDepth()`/`correctDvl()`（含精确杠杆臂补偿
`y_v_com = v_dvl_meas - omega_hat x r_dvl`）仅在采样保持值真正变化时触发修正
（内部自带"新样本"边沿检测，调用方无需知道硬件侧的 `50Hz`/`20Hz` 抽取相位）。
**设计取舍（已记录供复核）**：实现的是按通道解耦的固定增益（指数滑动平均
稳态 Kalman 形式）滤波器，而非单一联合协方差 EKF——增益用稳态方差公式
`Var(y_hat) = K/(2-K) * Var(meas)` 从 CLAUDE.md 给定的噪声标准差解析选取
（如 DVL `K=0.5` 对 `0.008 m/s` 输入噪声给出约 `0.0046 m/s` 稳态输出标准差），
在 `test_sensor_fusion_and_teleop.cpp` 中验证满足 `<0.015 m/s`/`<15mm`/
`<1.0deg` RMSE 契约，用户已确认该工程简化可接受。

`CtrlInterfaces.h` 删除 `odom_state_interface_`/`imu_state_interface_`/
`pad_contact_state_interface_`（地面真值）三个成员，新增 `ActiveState`
（`pos_w/rpy/vel_body/omega_body/pad_force/v_sep`）结构体与
`active_state` 公开成员——这是 `FSMState` 能读到的唯一位姿/速度/接触力来源。
`adhere_controller.hpp` 把原本写进 `CtrlInterfaces` 的地面真值
`LoanedStateInterface` 绑定改为 `AdhereController` 自己的私有成员
（`gt_imu_state_interface_`/`gt_odom_state_interface_`/
`gt_pad_contact_state_interface_`），并新增 4 个噪声传感器私有绑定
（`noisy_*_state_interface_`）。`update()` 在每个 500Hz tick 的内环门控**之前**
读取地面真值到局部变量；若 `estimation_mode=="sensor_fusion"`，驱动
`estimator_.predict()/correctAttitude()/correctDepth()/correctDvl()` 并把
融合结果写入 `active_state`；否则直接拷贝地面真值。地面真值局部变量**只**
用于填充外环发布的 `FsmState::ground_truth_pose`，从不写入 `active_state`。
`FsmState` 发布块新增 `estimation_mode` 字符串与 6 维 `estimated_pose`/
`ground_truth_pose`。

`FSMState.h::targetPose()` 从 4 维扩为 6 维；`StateDepthHold`/`StatePidHold`
的所有位姿/速度读取从 `ci.odom_state_interface_[i].get().get_value()` 改为
`ci.active_state.pos_w[i]`/`.rpy[i]` 等（新增 `mathTools.h::rpyToRotMat()`，
因为 `ActiveState` 存 RPY 而非四元数）。`StateOpenLoop` 把
`teleop_roll_rate`/`teleop_pitch_rate` 映射为 `Tx=max_wrench[3]*roll_rate`/
`Ty=max_wrench[4]*pitch_rate`（这两个 `max_wrench` 分量此前已存在但从未使用）。
`StatePidHold` 新增 `phi_d_`/`theta_d_` 保持目标，摇杆/扳机推动期间按
`max_teleop_vel[3]/[4]` 积分并钳制在 `±pid_hold.max_tilt_deg`，姿态误差项从
仅含 yaw 扩展为全部 3 轴；`SetpointCommand` 的 `mode==4` 边沿检测同时消费
`setpoint_target_roll/pitch`。`ModelParams.h` 新增 `SensorMount`
（`r_imu/r_depth/r_dvl`）、`PidHoldGains::max_tilt_deg`，
`max_teleop_vel` 由 `std::array<double,4>` 扩为 `<double,6>`。

### `adhere_keyboard_input`：LT/RT 俯仰/横滚遥操作 + 控制台 roll/pitch/reset

`xbox_teleop_node.cpp` 新增 `applyTriggerDeadzone()`（扳机单侧行程 `[静止
+1.0, 完全按下-1.0]` 先做 `(1-raw)/2` 归一化到 `[0,1]` 再应用死区，与摇杆的
双侧 `[-1,1]` 约定不同）、`axis_pitch_trigger`(LT,默认2)/`axis_roll_trigger`
(RT,默认5)/`button_pitch_reverse`(LB,默认4)/`button_roll_reverse`(RB,默认5)
5 个新参数；`pitch_rate`/`roll_rate` = LB/RB 按住时取负、否则取正的 LT/RT
幅值。`state_cmd_console_node.cpp` 的 `mode==4` 分支扩展 `roll <deg>`/
`pitch <deg>`/`reset`（roll/pitch 归零，x/y/z/yaw 原样保留）输入解析，
"Actual"/"Target" 显示扩展为 6 项；`FsmState` 订阅回调改用
`msg->estimated_pose`（而非已删除的 `actual_pose`）填充本地显示缓存。

### 验证结果

`export PATH=/usr/bin:$PATH && colcon build --symlink-install`（全部 5 个包）
**零警告零错误**。`colcon test`：新增的 `test_sensor_fusion_and_teleop`
（8/8：DVL 杠杆臂补偿收敛性、`StateDepthHold`/`StatePidHold` 纯
`sensor_fusion` 闭环跟踪、LT/RT 触发器映射单元测试、`StateOpenLoop`/
`StatePidHold` 俯仰/横滚映射）与既有 `test_fsm_states`（3/3，已改写为直接写
`ci.active_state` 而非假 `LoanedStateInterface`）、`test_multirate_decimator`
（4/4）、`test_thruster_allocator`（3/3）、`test_fossen_hydrodynamics_geometry`
（26/26）、`test_rhombic_kern_and_thruster_model`（9/9）**全部通过**；仅剩
Phase 2.14 起已记录的"全仓库缺版权头"等预先存在的
`copyright`/`cpplint`/`cppcheck`/`uncrustify`/`flake8`/`pep257`/`xmllint`
（`adhere_control_msgs/package.xml` 的既有 schema 问题，与本阶段无关）
style-lint 失败。

真实 MuJoCo 闭环（`ros2 launch simplified_v1_description mujoco.launch.py
headless:=true use_rviz:=false launch_teleop:=false
estimation_mode:=sensor_fusion`，经 `rclpy` 脚本驱动）：`DEPTH_HOLD` +
`SetpointCommand{target_z=-3.5}` 收敛后 `estimated_pose.z`/`ground_truth_pose.z`
均稳定在 `-3.50 ±0.01m` 附近且彼此互差 `<3mm`，`/rov/fsm_state` 的
`estimation_mode` 字段正确报告 `"sensor_fusion"`（未传参时默认
`"ground_truth"`），确认本阶段的 dual-mode 契约与 `estimation_mode` 启动参数
均在真实闭环中生效，而非仅通过纯 C++ 单元测试验证。

`grep -rn "gt_odom_state_interface_\|gt_imu_state_interface_\|
gt_pad_contact_state_interface_" src/controllers/adhere_controller/{src,
include}` 隔离审计：全部匹配均位于 `adhere_controller.cpp`/`.hpp`，
`FSM/*.cpp`/`FSM/*.h` 零匹配，确认地面真值隔离契约在代码层面成立。

## Phase 3.6（2026-10-18）：四元数消息重构 + Sim-to-Real 严格分离 + RuntimeDiagnostics 实时观测 + Pre-MPPI 基础设施

### 动机

Phase 4（`Baseline MPPI`/`KAMR-MPPI`）落地前还有三处架构缺口：（1）`FsmState.msg`
把仿真专属的 `ground_truth_pose` 和真实可部署的控制器状态混在一起，姿态用
Euler `float64[6]` 表示，存在万向锁奇异且与 Jetson 真实部署（没有地面真值）
不兼容；（2）完全没有量化手段确认各传感器/估计器/内外环是否达到目标频率、
有没有超出计算预算——25Hz CUDA MPPI 外环上线前必须先把这个观测能力建好，
因为它正是最可能超预算的环节；（3）接触力分布矩阵 `A_contact`、Rhombic
Kern 约束矩阵 `C_kern`、异步外环工作线程骨架都要在 Phase 4 把 MPPI 采样接进来
之前就存在。另外发现两个已有话题其实就是本阶段要加的新话题的雏形——
`/rov/wave_mode_cmd`（对应新 `/rov/flow_mode_cmd`）与 `/rov/odom`（对应新
`/rov/ground_truth_odom`）——经用户确认后选择**重命名**而非并存两套命名。

本阶段按照既有的子阶段提交规范拆成 3.6.1/3.6.2/3.6.3 三次独立的
build-test-commit 里程碑。

### Phase 3.6.1：消息重构 + 四元数 SO(3) 姿态控制

`adhere_control_msgs`：`FsmCommand.msg` 新增 `BASELINE_MPPI=5`/`KAMR_MPPI=6`
预留常量；`SetpointCommand.msg` 把 `mode/lock_current_pose/target_x..yaw`
替换为 `command_type`（`1=MODE_DEPTH_ONLY, 2=MODE_FULL_POSE,
3=MODE_LOCK_CURRENT, 4=MODE_LEVEL_ATTITUDE`）+ `geometry_msgs/Pose
target_pose`；`TeleopCommand.msg` 把 6 个裸标量替换为
`geometry_msgs/Vector3 linear`（surge/sway/heave）+
`geometry_msgs/Vector3 angular`（roll_rate/pitch_rate/yaw_rate）；删除
`FsmState.msg`，新增 `ControllerState.msg`（`/rov/controller_state`）——不再
含任何 `ground_truth_pose` 字段，100% Sim-to-Real 纯净。

`adhere_controller`：`mathTools.h` 新增 `quatMul`/`normalizeQuat`/
`smallAngleQuat`（从 `RovStateEstimator.h` 提升为公共函数，避免重复实现）、
`quatInverse`/`quatFromRPY`/`quatError`（`e_R=2·sgn(q_e.w)·[q_e.x,q_e.y,
q_e.z]`，`q_e=q_est^{-1}⊗q_d`）。`ActiveState` 新增 `quat_w[4]`（世界系姿态
四元数），`rpy[3]` 改为由 `quat_w` 派生，仅供 `StateDepthHold` 的小角度
Euler 阻尼使用（该状态从不跟踪大姿态目标，不存在万向锁风险，因此保留
Euler 写法而不强行改成四元数）。`ControlInputs` 的
`setpoint_mode/setpoint_lock_current_pose/setpoint_target_{x,y,z,roll,
pitch,yaw}` 替换为 `setpoint_command_type` + `setpoint_target_pos[3]` +
`setpoint_target_quat[4]`；`teleop_*` 字段内部原子量命名不变，只是订阅回调
改为从 `msg->linear.{x,y,z}`/`msg->angular.{x,y,z}` 取值。`FSMState::
targetPose()` 返回类型从 `std::array<double,6>` 改为新的 `PoseTarget{Vec3
pos; Quat quat;}`。`StatePidHold` 把 `phi_d_/theta_d_/psi_d_` 三个 Euler
目标替换为单一 `Quat q_d_`：摇杆/扳机角速度通过
`q_d_=normalize(q_d_⊗smallAngleQuat(ω_cmd·dt))` 积分，姿态误差改用
`quatError(q_est, q_d_)` 直接喂给既有的 `kp_att⊙e_R+ki_att⊙integral+
kd_att⊙e_omega` PD 律，±max_tilt_deg 钳制通过 RPY 往返转换实现（代价很低，
精确保持既有钳制契约）。`adhere_controller.cpp` 的地面真值/估计器融合分支
都改为直接写入 `active_state.quat_w`，不再在 `update()` 里把 ground truth
转成 Euler 发布——`/rov/controller_state` 发布块彻底移除了
`ground_truth_pose` 这一项。

`adhere_keyboard_input`：`xbox_teleop_node.cpp` 改发布 `linear`/`angular`
`Vector3`；`state_cmd_console_node.cpp` 新增本文件自带的轻量
RPY↔四元数小工具（不依赖 Eigen，与 `adhere_controller` 的 `mathTools.h`
同约定），友好的 `x/y/z` 米、`roll/pitch/yaw` 度输入体验保持不变，显示同时
给出四元数 `[qw,qx,qy,qz]` 与换算出的 RPY(deg)；`fsm_console_node.cpp` 订阅
改为 `ControllerState`。

验证：`colcon build --symlink-install`（5 个包）零警告零错误；`colcon
test --packages-select adhere_control_msgs adhere_controller
adhere_keyboard_input`：既有 + 更新后的 gtest 全部通过（18/18 用例，仅剩
Phase 2.14 起已记录的全仓库缺版权头等预先存在 lint 失败）。真实 MuJoCo
闭环（`rclpy` 脚本驱动）：`PID_HOLD` 的 `MODE_FULL_POSE` 设定点含非零
roll/pitch/yaw，25s 后 `estimated_pose`/`target_pose` 的位置与四元数均收敛
到彼此 `<0.06m` 以内。

### Phase 3.6.2：`LoopProfiler` + `RuntimeDiagnostics` + RViz HUD + `rt_monitor_node`

`adhere_control_msgs` 新增 `RuntimeDiagnostics.msg`（9 路频率 + 延迟/预算 +
超时计数 + `timing_healthy`，10Hz 发布）。`adhere_controller` 新增
头文件式 `common/LoopProfiler.h`（风格与 `MultiRateDecimator.h` 一致，无
ROS/Eigen 依赖）：`tickMarker()` 用 steady_clock 间隔的 EMA 估计触发频率，
`beginExec()/endExec()` 测最新/EMA均值/最大执行耗时并对预算计超时次数。

`adhere_mujoco_hardware`：`hw_states_` 从 55 扩到 57，新增
`diagnostics/hw_loop_hz`/`diagnostics/hw_step_exec_ms` 两个状态接口——直接
镜像既有 `MujocoVisBridge::actual_physics_hz`/`step_work_ms`（物理线程早已
用 0.98/0.02 EMA 算好的遥测），而不是在硬件侧再起一份 `LoopProfiler`
实例重复测量同一件事，也避免了 `adhere_mujoco_hardware` 反向依赖
`adhere_controller`。同时把 `/rov/wave_mode_cmd`→`/rov/flow_mode_cmd`、
`AdhereMujocoHardware::setWaveMode/waveMode`→`setFlowMode/flowMode`、
`MujocoVisBridge` 的 `wave_mode_cmd/wave_mode_active`→
`flow_mode_cmd/flow_mode_active`、`MujocoViewer` 的 `ui_wave_mode_` 等全部
改名为 `flow`（与 CLAUDE.md 自己的 `FlowEnvironmentPlugin` 术语一致）；
`/rov/odom`→`/rov/ground_truth_odom`（`adhere_mujoco_node.cpp` 的发布者
改名，`mujoco_sim.rviz` 的 Odometry 显示同步改名）。`fossen_hydrodynamics.hpp`/
`flow_environment_plugin.hpp` 里描述真实物理"wave-impact gust"现象的
"wave" 字样保持不动——那是合法的水动力学术语，与被改名的"模式切换"概念
无关。

`adhere_controller`：绑定新增的 2 个诊断状态接口；新增
`estimator_profiler_`/`inner_profiler_`（4.0ms 预算）/
`outer_profiler_`（40.0ms 预算）/`sensor_imu_profiler_`/
`sensor_loadcell_profiler_`/`sensor_depth_profiler_`/`sensor_dvl_profiler_`
7 个 `LoopProfiler` 实例；IMU/称重传感器每 tick 调用 `tickMarker()`，
深度/DVL 用与 `RovStateEstimator` 同款的"采样保持值是否真的变化"边沿检测
触发。每 50 个 500Hz tick（独立于 `inner_decimation_`/`outer_decimation_`，
固定 10Hz）调用新增的 `publishRuntimeDiagnostics()`：发布
`/rov/runtime_diagnostics`，并发布一个 `visualization_msgs::Marker`
（`TEXT_VIEW_FACING`，健康绿/异常红）到 `/rov/diagnostics_hud_marker`，
浮在 ROV 正上方 0.6m。`timing_healthy` 故意把 `estimator_hz` 排除在频率检查
外——`estimation_mode=="ground_truth"` 时估计器本就不跑，报 0Hz 并非超时。

`adhere_keyboard_input` 新增 `rt_monitor_node`（ANSI 彩色实时表格，超时/
偏离目标标红）与 `fsm_console_node` 的单行计时摘要。

验证：`colcon build --symlink-install`/`colcon test`（22/22 gtest 用例，含
新增 `test_loop_profiler`）全部通过；真实 MuJoCo 闭环确认 8 路频率全部落在
目标值 `±10%` 内（500/500/500/50/20/250/25/250Hz）、零超时、
`timing_healthy=true`，HUD marker 渲染为绿色，`rt_monitor_node` 实时表格
正确，`ros2 topic pub /rov/flow_mode_cmd std_msgs/msg/Int32 "data: 2"`
成功无 GUI 触发 12.0s 阵风基准。

### Phase 3.6.3：`A_contact`/`C_kern` 预计算 + 异步外环工作线程骨架

`ThrusterAllocator.h` 新增 `buildPadNormalMap()`（`G∈R^{3×4}`）、
`buildPadNormalMapPseudoInverse()`（`G^†∈R^{4×3}`）、`buildContactMap()`
（`A_contact=-G^†B_{[Fx,Ty,Tz]}∈R^{4×8}`）、`padHalfSpans()`、
`buildKernConstraintMatrix()`（`C_kern∈R^{4×8}`，4 个符号组合边约束）。
**发现并修正的符号问题**：CLAUDE.md 文字描述 `G` 第一行系数为 `-1`，但其
自带的验证算例（`u[0:4]=25N` 等幅压紧推力应得到
`A_contact·u=[25,25,25,25]^T N`）用数值验证后只有系数取 `+1` 才成立——
两者矛盾，以"显式给出的数值算例"为准，`buildPadNormalMap()` 实现为 `+1`
并在代码注释与 `test_thruster_allocator.cpp` 新增的
`ContactMapReproducesEqualPressingLoadExactly`/
`KernConstraintSatisfiedForSymmetricPressingLoad` 两个用例里留痕说明。
`ModelParams` 新增 `kern_eps_margin`（默认 `0.20`）。`AdhereController::
on_activate()` 与各 `FSMState` 构造 `B`/`B^†` 同一时机预计算
`a_contact_`/`c_kern_`/`kern_l_y_`/`kern_l_z_`；`/rov/controller_state` 的
`contact_forces`/`kern_margin` 改为由 `a_contact_ * u_cmd`（已下达的推力
指令）估计，而非直接读测量贴垫力传感器——这正是未来 KAMR-MPPI 的 12D
OSQP 约束要用的同一套准静态预测。`C_kern` 预计算完毕但暂未接入任何实时
约束（留给 Phase 4 的 OSQP）。

异步外环工作线程骨架：`AdhereController` 新增
`outerWorkerLoop()`（在独立 `std::thread` 里跑）、
`OuterLoopInput`/`OuterLoopOutput` 双缓冲结构体（`mutex`+
`condition_variable` 保护，不用 `realtime_tools::RealtimeBuffer`——那种
无锁 SPSC 结构要等 worker 真正按外环预算跑起来才有意义，骨架阶段一把
mutex 足够简单可靠）。线程在 `on_activate()` 启动、`on_deactivate()`/析构
函数里停止并 `join()`。每个 `tick.outer_due` 把当前 `active_state` +
`targetPose()` 快照写入输入缓冲并 `notify_one()`；worker 线程目前只做
"回传 seq，不做任何真实求解"的占位处理——Phase 4 只需替换 worker 循环体里
的求解逻辑，线程生命周期/握手架构本身不用再改。

验证：`colcon build --symlink-install`（5 个包）零警告零错误；`colcon
test --packages-select adhere_controller`：24/24 gtest 用例全部通过
（含 `test_thruster_allocator` 新增的 2 个 Kern 验证用例）；真实 MuJoCo
闭环确认 `list_controllers` 显示 `adhere_controller` 正常 `active`（线程
启动未报错）、`/rov/controller_state`/`/rov/runtime_diagnostics` 均正常
发布、`kill -INT` 后进程在 3s 内全部干净退出（`outer_worker_thread_`
无死锁）。

## Phase 3.7（2026-10-18）：`adhere_controller` 目录模块化、单一配置源、
估计器/输出安全硬化

### 动机

Phase 4（`Baseline MPPI`/`KAMR-MPPI`）落地前，`adhere_controller` 的
`include/.../common/` 已经变成一个大杂烩——动力学参数（`ModelParams.h`）、
控制分配矩阵（`ThrusterAllocator.h`）、计时工具（`LoopProfiler.h`/
`MultiRateDecimator.h`）、数学工具（`mathTools.h`）全混在一起，且
`RovStateEstimator`/`ThrusterAllocator`/`ModelParams` 全部以内联头文件实现，
PID 控制律直接写死在 `StateDepthHold.cpp`/`StatePidHold.cpp` 里。随着后续要
陆续加入 `pid`/`wbc`/`mppi` 等多种控制/规划算法，必须先把 `include/` 与
`src/` 拆成干净、可扩展的子目录层级，让 `.cpp` 编译单元与头文件分离。同时
发现两处重复/错放的配置文件（`adhere_controller/config/controllers.yaml`、
`adhere_mujoco_hardware/config/mujoco_sim.rviz`）需要清理，以及三处
缺失的硬化点：控制器侧缺一个可调用的标称 `FossenDynamics` 算子、
`RovStateEstimator` 缺少接触辅助的伪测量更新、四元数插值/误差缺少显式的
半球对齐、`AdhereController::update()` 写出推力器指令前缺少最终的
NaN/Inf 安全闸门。

### 目录重组（`include/adhere_controller/` 与 `src/`）

新增 `dynamics/`（`ModelParams.h`/`FossenDynamics.h` + 对应 `.cpp`）、
`allocation/`（`ThrusterAllocator.h` + `.cpp`）、`estimator/`
（`RovStateEstimator.h` + `.cpp`）、`algorithms/pid/`
（`SixDofPidController.h` + `.cpp`）、`algorithms/wbc/`
（`WbcAllocatorBase.h`，纯抽象数据契约，无实现）、`algorithms/mppi/`
（`MppiPlannerBase.h`，同样纯抽象）五个新子目录；`ModelParams.h`/
`ThrusterAllocator.h` 从纯头文件内联实现改为"声明头文件 + `.cpp`
实现"，机械搬移原有逻辑，数值行为不变（`colcon test` 全部既有 gtest 回归
验证零差异）。`common/` 收敛为真正轻量、无状态的工具集：`enumClass.h`/
`mathTypes.h`/`mathTools.h`/`LoopProfiler.h`/`MultiRateDecimator.h`，新增
`OutputSafety.h`（见下）。`CMakeLists.txt` 的 `add_library()` 源文件列表、
所有 `#include` 路径同步更新。

### `FossenDynamics`（新增，`dynamics/FossenDynamics.h`+`.cpp`）

标称 Fossen 6-DOF 刚体+附加质量模型，CoM 位于 body 原点、惯量主轴对齐的
对角质量矩阵假设（与只读参考仓库 `~/Research/mujoco_mppi_ws/src/env/
hydrodynamics.py` 同样不建模离轴耦合一致）：`M=diag(m,m,m,Ixx,Iyy,Izz)+
diag(added_mass_lin,added_mass_ang)`（`ModelParams` 新增
`dynamics.{inertia_diag,added_mass_lin,added_mass_ang}` 三个 YAML 字段）；
`coriolisWrench(ν)` 用对角质量矩阵的闭式反对称 Coriolis-离心矩阵
（Fossen《Handbook of Marine Craft Hydrodynamics and Motion Control》
式 6.43），反对称性由构造保证，`ν^T C(ν)ν≡0` 恒成立；`dampingWrench(ν)`
直接复用既有的 `D_lin/D_quad/D_ang_lin/D_ang_quad`；`computeRestoringWrench
(q)` 把原先写死在 `StatePidHold.cpp` 里的浮力/重力恢复力矩公式原样搬出
（真实验证：`rho_water*gravity*displaced_volume-vehicle_mass*gravity`
在默认参数下算出 `+1.962N`，与 `StatePidHold.cpp` 旧注释记录的数值吻合，
新增 gtest 直接断言这个真实计算值，而非盲目断言任务描述里给出的占位
示例数字）；`forwardDynamics(ν,q,τ)=M^-1(τ-C(ν)ν-D(ν)ν-g(q))`。

### `algorithms/pid/SixDofPidController`（新增）

从 `StatePidHold.cpp` 里抽取出误差→力矩律：半球对齐
（`mathTools.h::alignQuaternionHemisphere()`）后再调用 `quatError()`
求 SO(3) 姿态误差、条件积分抗饱和（某轴积分已达到其限幅且新误差会继续
把它推得更远时，直接冻结该轴积分，而不是事后硬限幅）、测量值微分
（Kd 作用在估计速度/角速度上，从不对设定点做差分，杜杆设定点阶跃不会
产生微分冲击）、通过注入的 `FossenDynamics` 做浮力/阻力前馈。
`StateDepthHold`（Phase 3.7）改为用同一个 `SixDofPidController`
跑一个精简的"仅深度"模式：每拍把横向 X/Y 目标重新锚定到当前估计位置
（零横向控制权限，与原实现完全一致）、姿态目标设为单位四元数（等价于
原来的 `-kp_att*rpy-kd_att*omega` 小角度阻尼律，因为 `quatError(q_est,
identity)` 在小角度下约等于 `-rpy`），并额外获得了原来完全没有的浮力
前馈——这是一处刻意的行为改进，不是回归，已在真实 MuJoCo 闭环中验证
（`DEPTH_HOLD` 从 Z=-2.0m 收敛到目标 -3.0m，误差 <6mm）。`StatePidHold`
保留设定点积分（摇杆驱动的推进、离散 `MODE_*` 处理）在 FSM 层，只把
误差→力矩律本身委托出去（真实闭环验证：`PID_HOLD` 收敛到
`[0.3,0.2,-2.5]` 设定点，三轴误差均 <5mm）。

### `algorithms/wbc/WbcAllocatorBase.h` 与 `algorithms/mppi/MppiPlannerBase.h`（新增）

纯抽象数据契约头文件，无实现，标记 Phase 4 的落地点：
`NominalWrenchCmd`/`WbcAllocationResult`（12D OSQP 分配器契约）、
`MppiPlannerBase::solve(ActiveState, PoseTarget)->NominalWrenchCmd`
（异步 CUDA MPPI 外环契约，复用既有 `OuterLoopInput`/`OuterLoopOutput`
握手架构里已有的快照字段，替换 `outerWorkerLoop()` 函数体即可接入，
无需再改线程/握手架构本身）。

### `mathTools.h` 新增半球对齐与阶跃限幅工具

`alignQuaternionHemisphere(q_ref,q)`：`q_ref·q<0` 时翻转 `q` 的符号——
除了供 `SixDofPidController` 在求姿态误差前显式对齐外，也修正了
`RovStateEstimator::correctAttitude()` 的 nlerp 混合里原先手写符号检查
的写法（改为复用这个共享原语，行为不变，但消除了重复实现）。新增通用
`clampStepVec3()`/`clampSlewQuat()` 阶跃限幅原语（基于
`quatError()`+`smallAngleQuat()`，奇异点自由），本阶段只在新增 gtest 里
验证，未接入现有 FSM 设定点逻辑（避免引入行为漂移）。

### `RovStateEstimator` 接触辅助墙面约束更新

新增 `ModelParams.contact_force_thresh`（默认 `2.0N`，新 YAML 顶层键）。
新增 `RovStateEstimator::correctContact(pad_force_meas[4], thresh,
x_wall_contact)`，由 `AdhereController::update()` 在 `sensor_fusion`
模式下每拍传入 4 路原始 `loadcell_noisy` 读数调用：内部对 4 路做 EMA
滤波，仅当 4 路滤波后力值全部超过阈值时，才把 `v_hat_body_.x()→0`、
`p_hat_world_.x()→workspace_x_min`（复用既有的墙面最小站位参数作为接触
平面代理，不引入新几何参数）做伪测量混合更新，消除长时间贴墙吸附时
DVL 积分漂移。

### 输出安全闸门

`FSMState::run()` 不再直接调用 `.set_value()` 写推力器——改为写入
`CtrlInterfaces` 新增的 `pending_thruster_cmd[8]`。新增头文件式
`common/OutputSafety.h::sanitizeAndWriteThrusterCommands()`：检测到任意
一路 NaN/Inf，8 路全部清零并自增违规计数；否则逐路饱和到
`[thruster_min,thruster_max]`。`AdhereController::update()` 在
`tick.inner_due` 内、`state.run()` 之后唯一调用一次，是推力器指令写出
硬件前的集中式闸门；违规时节流 `RCLCPP_ERROR`。全部 4 个 FSM 状态 `.cpp`
与测试用例的假 `CtrlInterfaces` 夹具同步改为先调用这个闸门再读取写入
的推力器值。

### 配置文件去重

`git rm adhere_controller/config/controllers.yaml`——确认是死文件：只装载
`mujoco.launch.py` 从未实际加载的 3 行 `controller_manager` 元信息，真实
参数全部只存在于 `robot_control.yaml` 的 `adhere_mujoco_node` 键下；
同步移除 `adhere_controller/CMakeLists.txt` 里对应的 `install(DIRECTORY
config...)` 规则与空目录。`git mv adhere_mujoco_hardware/config/
mujoco_sim.rviz` 到 `simplified_v1_description/config/mujoco_sim.rviz`，
同步移除 `adhere_mujoco_hardware/CMakeLists.txt` 里的 `config` 安装规则
与空目录，`mujoco.launch.py` 的 RViz 配置路径解析挪到已有的
`pkg_description_share`（`OpaqueFunction` 内部），不再从
`adhere_mujoco_hardware` 的 share 目录读取任何文件——该包现在零模型/
可视化配置文件。

### 验证结果

`colcon build --symlink-install`（5 个包）零警告零错误；`colcon test
--packages-select adhere_controller`：35/35 gtest 用例全部通过（既有
24 个 + 新增 `test_dynamics_and_pid.cpp` 11 个：`FossenDynamics` 质量
矩阵对称正定/Coriolis 反对称二次型恒零/阻尼耗散非负/浮力恢复力精确值，
`SixDofPidController` 四元数半球不变性（`<1e-12`）与条件抗饱和限幅，
`RovStateEstimator` 接触辅助更新的漂移约束 vs. 无接触时自由漂移对比，
`OutputSafety` 的 NaN/Inf 全零化 vs. 有限越界值仅饱和不清零）；lint 结果
与既有基线一致（仅剩全仓库预先存在的缺版权头、cppcheck 对 `.h` 文件误判
为 C 代码等 Phase 2.14 起已记录的预先存在失败，新增文件本身零 uncrustify
偏差）。真实 MuJoCo 闭环（`mujoco.launch.py headless:=true use_rviz:=
false`）确认：`list_controllers` 显示 `adhere_controller` 正常 `active`；
`DEPTH_HOLD`（`MODE_DEPTH_ONLY` 目标 Z=-3.0m）从 Z=-2.0m 收敛到
`Z=-2.994m`（<6mm 误差）；`PID_HOLD`（`MODE_FULL_POSE` 目标
`[0.3,0.2,-2.5]`）收敛到 `[0.300,0.200,-2.497]`（三轴误差均 <5mm）；
进程 `SIGINT` 后干净退出，无残留。

## Phase 3.8（2026-10-18）：`ADHERE_PID`/`ADHERE_SMC` 贴壁基线控制器 + 效率/啁啾/安全遥测 + Benchmark 评估节点

### 动机

Phase 4（`Baseline MPPI`/`KAMR-MPPI`）落地前还缺两类"公平"的经典贴壁基线：现有
`PID_HOLD` 是自由悬浮的 6DOF 位姿保持，从不主动调节壁面法向预压力
（`Fx<0` 朝墙方向），不能作为贴壁站位的公平基线；同时论文/实验验证需要一套
量化的能耗/啁啾/饱和/Rhombic Kern 安全裕度/跟踪误差评估手段，才能定量展示
未来 MPPI/KAMR-MPPI 相对经典控制的优势。本阶段新增两个贴壁基线 FSM 状态
`ADHERE_PID`（ID `5`）、`ADHERE_SMC`（ID `6`），`FsmCommand`/`ControllerState`
的 `BASELINE_MPPI`/`KAMR_MPPI` 相应从 `5`/`6` 顺延到 `7`/`8`；新增集中式
效率/安全遥测字段与独立的 `benchmark_evaluator_node` 评估节点。

### `ADHERE_PID`（`algorithms/pid/AdherePidController` + `FSM/StateAdherePid`）

两阶段结构：APPROACH 阶段（未贴壁时）把 `p_d_world_.x()` 以
`adhere_pid.approach_speed` 匀速逼近 `adhere_pid.x_wall`，6DOF 全部由既有
`SixDofPidController` 驱动（与 `PID_HOLD` 完全相同的位置保持律）；一旦
4 个贴垫法向力估计之和超过 `adhere_pid.dock_force_thresh` 且位置进入
`adhere_pid.x_dock_eps` 范围内，状态切换为粘滞的 ADHERED 阶段（一旦进入
永不自动退出，只能通过 `/rov/fsm_command` 显式切换状态离开）：此后每拍把
`p_d_world_.x()` 重新锚定为当前估计位置（让 `SixDofPidController` 的 X 分量
贡献恒为零），再用新增的 `AdherePidController`（**不是** `SixDofPidController`
的包装，是独立的标量力 PI 调节器）覆盖写入 wrench 的 Fx 分量：
`A = f_preload + Kp_F(F_des-ΣF_N) + Ki_F·∫(...)dt`，
`Fx_wrench = -A - Kd_x·max(0,v_x)`——符号约定与 CLAUDE.md Sec.5.2 既有的
`-D_vx·max(0,v_x)` 阻尼项完全一致（后退越快，压墙力越大）。Y/Z/姿态滑动
（setpoint/teleop）与 `PID_HOLD` 同构，贴壁后 X 分量被忽略。

### `ADHERE_SMC`（`algorithms/smc/SixDofSmcController` + `FSM/StateAdhereSmc`）

新增 `SixDofSmcController`：误差约定采用"估计减期望"（`e_p_body=R^T(p_est-p_d)`，
`e_R_body=quatError(q_d,q_est)`，刻意与 `SixDofPidController` 的
`quatError(q_est,q_d)` 调换参数顺序使两者符号一致，详见该文件头注释里记录的
推导），`s=e_nu+Λ·e_eta`，控制律
`τ=C(ν)ν+D(ν)ν+g(q)-MΛe_nu+τ_wall_preload-Kd·s-Ksw·sat(s/φ)`（前三项通过
`FossenDynamics` 精确抵消标称动力学，`τ_wall_preload` 仅在 `docked_` 时叠加）。
与 `ADHERE_PID` 不同，`StateAdhereSmc` **全程只用一个** `SixDofSmcController`
实例驱动全部 6DOF（不为 APPROACH 阶段单独构造位置控制器）：其自身的位置误差项
天然驱动逼近，贴壁后只是额外叠加一个常值压墙偏置。贴壁判定、setpoint/teleop
滑动逻辑与 `ADHERE_PID` 完全镜像。

### 效率/啁啾/安全遥测（`common/ControllerMetrics.h` + `ControllerState.msg`）

`ControllerState.msg` 新增 7 个字段：`thrust_l2_norm`（`‖u‖_2`）、
`thrust_power_proxy`（`Σ|u_i|^1.5`，CLAUDE.md Sec.5 功率代理公式）、
`thrust_slew_rate_l2`（`‖(u[k]-u[k-1])/dt‖_2`，啁啾指标）、
`cumulative_energy_l2`（`∫‖u‖_2^2 dt`）、`cumulative_impulse_l1`（`∫‖u‖_1 dt`）、
`saturated_thruster_count`、`active_contact_pad_count`。新增头文件式
`common/ControllerMetrics.h::ControllerMetricsAccumulator`（风格与
`OutputSafety.h`/`LoopProfiler.h` 一致，无 ROS 依赖，独立可测），由
`AdhereController::update()` 在每个 250Hz 内环拍从**真正发出**（经
`sanitizeAndWriteThrusterCommands()` 清洗/饱和后、通过
`thruster_command_interface_[i].get_value()` 回读）的推力器指令计算，FSM
状态切换时（`mode_==FSMMode::CHANGE` 分支）调用 `reset()` 清零累积积分。
饱和判定用每个推力器自己的 `ModelParams::thruster_max[i]`（而非任务描述文字
里字面给出的 `60N`——该值与 `robot_control.yaml` 实际配置的 `40N` 不一致，
按 CLAUDE.md Sec.3.3 的零硬编码规则取动态参数而非魔法数）。

### `ModelParams`/`robot_control.yaml`

新增 `ModelParams::AdherePidGains`/`AdhereSmcGains` 两个嵌套结构体，镜像
既有 `PidHoldGains` 的 YAML 加载模式（`ModelParams.cpp::loadFromNode()` 新增
`declareVec6()` 辅助函数，供 `AdhereSmcGains` 的 6 维滑模增益数组使用）；
`robot_control.yaml` 新增 `adhere_pid.*`/`adhere_smc.*` 两个参数块，紧跟在
既有 `pid_hold.*` 块之后。

### `adhere_keyboard_input`：控制台/监控扩展 + `benchmark_evaluator_node`

`fsm_console_node` 的 `stateLabel()`/菜单/键盘输入解析/自动弹出设定终端逻辑
扩展支持 `5`/`6`（`ADHERE_PID`/`ADHERE_SMC`），并新增一行效率/安全遥测摘要
（从 `/rov/controller_state` 更新）。`state_cmd_console_node` 的 `mode`
参数扩展接受 `5`/`6`，与既有 `mode==4`（`PID_HOLD`）完全复用同一套
`z/y/x/roll/pitch/yaw/all/reset/h` 命令语法——贴壁后 `x` 分量在 FSM 层
本就被忽略，无需新语法。`rt_monitor_node` 新增对 `/rov/controller_state`
的订阅，在既有 9 路频率诊断表下方追加一张效率/安全表（`||u||`/功率代理/
啁啾/累积能量/饱和推力器数/贴壁垫数/Kern 裕度，红绿着色）。新增独立节点
`benchmark_evaluator_node`（纯 `rclcpp::Node`，不 fork 子进程，风格与
`rt_monitor_node` 的定时重绘一致）：订阅 `/rov/controller_state`、
`/rov/ground_truth_odom`（本包中唯一允许触达该离线评估专用话题的节点）、
`/rov/runtime_diagnostics`，用一个可配置的滚动窗口（默认 `12.0s`）聚合出
跟踪精度（Y/Z RMSE/最大滑移、姿态 RMSE/最大误差，均相对地面真值）、控制
能耗（`E_u`/`I_u`/平均功率代理）、平顺性（滑移率 RMS/峰值、饱和占比）、
贴壁安全性（最小 Kern 裕度、Kern 违规占比、4 垫满接触保持率）的
Markdown 表格，可选 `trigger_flow_gust:=true` 参数在启动时单次触发
`/rov/flow_mode_cmd=2`（12s 阵风基准场景）。`CMakeLists.txt`/`package.xml`
新增 `nav_msgs`/`std_msgs` 依赖（核心 ROS 2 包，不违反零 MuJoCo/CUDA/
ros2_control 依赖规则）。

### 验证结果

`export PATH=/usr/bin:$PATH && colcon build --symlink-install`（5 个包）
零警告零错误。`colcon test`：`adhere_controller` 新增 `test_adhere_wall_states`
（3 个用例：`StateAdherePid`/`StateAdhereSmc` 各自对一个简化单边弹簧
贴墙接触模型收敛到 4 垫均正接触力 + 真实 `C_kern` 约束验证通过 + Y/Z
设定点保持，以及一个对比用例——在相同持续 +Y 扰动下 `StateAdhereSmc`
的最大位置误差小于 `StateAdherePid`，而紧边界层/高切换增益配置下的
`StateAdhereSmc` 观测到的推力滑移率 RMS 显著高于 `StateAdherePid`，
用真实闭环动力学而非断言验证了"SMC 抗扰更强但更啁啾"的预期物理权衡）
与 `test_controller_metrics`（6 个用例：首拍零滑移率、滑移率手算验证、
累积能量/冲量单调递增且与手算积分吻合、`reset()` 清零累积量且不产生
跨状态切换的虚假滑移、饱和计数在 `0.98×thruster_max` 边界触发、接触垫数
按 `contact_force_thresh` 判定）全部通过，加上既有 44 个用例零回归
（5 个新增用例文件 + 既有全部套件共 44/44）；lint 结果与既有基线一致
（仅剩全仓库预先存在的缺版权头等 Phase 2.14 起已记录的失败）。真实 MuJoCo
闭环（`mujoco.launch.py headless:=true use_rviz:=false
estimation_mode:=ground_truth`，经 `ros2 topic pub/echo` 驱动）确认：
切换到 `ADHERE_PID`（`command:=5`）与 `ADHERE_SMC`（`command:=6`）均不
崩溃，`/rov/controller_state` 正确发布 `current_state=5/6`、
`state_name=state_adhere_pid/state_adhere_smc`，贴垫接触力随进程运行从
0 增长到正值（该 ROV 默认出生点 `X=0.125m` 本就非常接近墙面
`x_wall=0.0m`，APPROACH 阶段耗时很短），新增的 7 个效率遥测字段与
`kern_margin` 均正常输出合理数值；进程通过 `SIGTERM` 干净退出。

## Phase 4.1（2026-10-18）：纯 CUDA 外环 Baseline MPPI (`BASELINE_MPPI`) + 算法专属动力学架构

### 动机

Phase 4 要落地完整的 `KAMR-MPPI` 之前，需要先有一个干净的纯 CUDA 采样式规划器
基线，用作消融对照：`BASELINE_MPPI`（ID `7`）是**仅外环**（无 250Hz 内环 PID/
DOB/QP）的 CUDA MPPI 实现，同时借此确立"每种规划算法拥有自己的动力学/配置
表示"的架构模式（`dynamics/ModelParams` 只保留物理真值，不再被某个具体算法
的超参数污染），为下一阶段 `kamr_mppi` 落地接触耦合动力学铺路。

### `algorithms/baseline_mppi/`：零 Eigen/ROS 依赖的 GPU 类型 + 独立配置解析器

新增子目录，与既有 `pid/`/`smc/`/`wbc/`/`mppi/` 并列：`BaselineCudaTypes.h`
定义纯 `float` POD 结构体（`CudaState6Dof`/`CudaBaselineModelParams`/
`CudaBaselineRuntimeConfig`），`BaselineMppiKernels.h` 是 host（g++）与
device（nvcc，`baseline_mppi_kernels.cu`）之间唯一共享的头文件——故意不含任何
`curand_kernel.h` 类型或核函数启动语法，Philox RNG 状态数组对 host 侧始终是
不透明的 `void*`。`BaselineMppiConfig`（`K/H/dt/lambda/sigma_tau/Q_*/R_*/
x_wall/wall_k/wall_b/preload_fx/seed`）由独立的 `BaselineMppiConfig::
loadFromNode()` 解析 `robot_control.yaml` 新增的 `baseline_mppi.*` 键块，
**不**挂在 `ModelParams` 下，与 `ModelParams` 只保留物理真值参数的既有原则
一致。

### `BaselineMppiController`：持久化设备缓冲区 + 4 核函数流水线

`BaselineMppiController::init()`（`AdhereController::on_activate()` 中与
`a_contact_`/`c_kern_` 同一时机调用，复用同一套 `B`/`B^+`，新增成员
`b_`/`b_pinv_` 缓存）一次性 `cudaMalloc`/`cudaHostAlloc` 全部设备/锁页宿主
缓冲区（`eps`/`costs`/`weights`/`tau_bar`/`tau_bar_new`/`u0`），`solve()`
之后零运行时分配。4 个核函数（而非任务原文的 3 个——CUDA 核函数之间没有
跨 block 的全局屏障，"平滑+右移+求输出"必须拆成独立的第 4 个
`finalizeNominalKernel`）在一条 `cudaStreamNonBlocking` 流上依序提交：
`rolloutBaselineMppiKernel`（K=1024 路并行 RK2 积分 `stepBaselineFossen6Dof`
+ 四元数代价累积）→ `softmaxWeightsKernel`（两段式 shared-mem+warp-shuffle
归约）→ `updateNominalKernel`（每时域步一个 block 的加权求和归约）→
`finalizeNominalKernel`（3 点平滑 + 热启动右移 + `B^+` 求 `u_0*`）。
`StateBaselineMppi`（`FSM/StateBaselineMppi.{h,cpp}`）的 `enter()` 锁存当前
位姿为目标并调用 `warmStart()`（重置内部名义序列为静水恢复力前馈
`-g(q_d)`），设定点/摇杆积分与 `StatePidHold` 同构；`run()` 在 250Hz 严格零阶
保持，只把 `CtrlInterfaces::outer_loop_cmd_`（新增的无锁 `OuterLoopCommand`
桥，与 `ActiveState` 同样的隔离纪律）逐值写入 `pending_thruster_cmd`，不含
任何内环控制律。`BaselineMppiController::solve()` 直接返回 8 维推力器指令，
**不**实现 `MppiPlannerBase`/`WbcAllocatorBase`（这两个纯数据契约抽象类仍
专属 Phase 4.2 的 `KAMR-MPPI`）。

### 调试记录：softmax 归约对单条 rollout 发散的脆弱性（真实 bug，已修复）

开发过程中在真实 GPU（本工作站 RTX5000）上复现过一次严重 bug：切换到
`BASELINE_MPPI` 后，无论目标位姿如何变化，8 个推力器指令恒为 `-40N`
（全饱和、与目标无关）。逐层打桩定位（CPU 侧重新计算 `eta` 并与 GPU 结果
比对）确认：`K=1024` 条 rollout 里只要有**一条**在显式 RK2 积分下数值发散
（陡峭的贴墙弹簧阻尼 `wall_k=4000`/二次阻尼与较大的探索噪声 `sigma_tau`
叠加，在 `H=30`/`dt=0.04s` 的时域内小概率发散到 `Inf`/`NaN`），其 `cost`
变为 `NaN` 后会通过 `costs[k]-rho`/`expf()` 污染 `softmaxWeightsKernel`
里**所有** `K` 条 rollout 共享的 `eta` 累加——`fmaxf(NaN, 1e-30f)` 悄悄把
`NaN` 吞成 `1e-30`，看起来像"正常但权重异常尖锐"，实际上整条
`tau_bar_new` 变成 `NaN`，经 `clamp()` 后又被 IEEE `fmin`/`fmax` 的 NaN
处理规则统一钳到 `u_min=-40N`。修复为双重数值安全网：`rolloutBaselineMppi
Kernel` 每步后把速度/角速度钳制在 `±50`（物理上远超这台 ROV 的任何正常
工况），且终值用 `isfinite()` 兜底替换为大而有限的哨兵代价（`1e12`）——
两者任一生效都能保证单条发散的 rollout 只会被软最大值赋予趋近于零的权重，
而不会把 `NaN`/`Inf` 带进所有 `K` 条共享的归约。此 bug 修复后 `test_baseline_
mppi.cpp` 的三个 gtest 从全部失败转为全部通过。

### 诊断集成与 CMake CUDA 构建

`RuntimeDiagnostics.msg` 新增 `outer_planner_solve_ms`（无 GPU 规划器激活时
恒为 `0`），数据来自 `BaselineMppiController` 自带的 `LoopProfiler`，经
`AdhereController::update()` 在 `outer_worker_mutex_` 保护下与
`ctrl_interfaces_.outer_loop_cmd_` 一起拷贝到 500Hz 线程侧；
`rt_monitor_node` 的诊断表格新增一行展示该延迟。`adhere_controller/
CMakeLists.txt` 新增 `enable_language(CUDA)` + `CMAKE_CUDA_ARCHITECTURES
"72;75"`（严格匹配 CLAUDE.md 第 2 节的 Jetson AGX Xavier/RTX5000 目标，不
采用任务描述字面更宽的 `72;75;80;86`）；本工作站 `nvcc` 不在
`$PATH`（CLAUDE.md 第 0 节的构建命令只前置 `/usr/bin`），故在
`CMAKE_CUDA_COMPILER` 未被外部设置时自动回退到 `/usr/local/cuda/bin/nvcc`；
本工作站 CMake 3.16 早于 `FindCUDAToolkit`（需 3.17+），改用传统
`find_package(CUDA REQUIRED)` 模块提供 `CUDA_INCLUDE_DIRS`/
`CUDA_LIBRARIES`/`CUDA_curand_LIBRARY`。`-Wall`/`-Wextra`/`-Wpedantic`
通过 `$<COMPILE_LANGUAGE:CXX>` 生成器表达式限定只作用于 C++ 源文件，避免
nvcc 收到它不认识的 host 编译器选项直接报错退出。

### `adhere_keyboard_input` 扩展

`fsm_console_node` 的菜单/键盘解析/动态子终端弹出逻辑扩展支持 `7`
（`BASELINE_MPPI`）；`state_cmd_console_node` 的 `mode` 参数扩展接受 `7`，
与既有 `mode==4/5/6` 完全复用同一套 `z/y/x/roll/pitch/yaw/all/reset/h`
命令语法；`rt_monitor_node` 新增 GPU 规划器求解延迟行。`xbox_teleop_node`
的 4 个面板按键（A/B/X/Y）已被 `PASSIVE`/`PID_HOLD`/`DEPTH_HOLD`/
`OPEN_LOOP` 占满——与 Phase 3.8 的 `ADHERE_PID`/`ADHERE_SMC` 保持一致，
`BASELINE_MPPI` 同样不绑定手柄按键，只能通过 `fsm_console_node` 键盘切换。

### 验证结果

`export PATH=/usr/bin:/usr/local/cuda/bin:$PATH && colcon build --symlink-
install`（5 个包，含新增 CUDA 编译步骤）零警告零错误。`colcon test`：
`adhere_controller` 新增 `test_baseline_mppi`（3 个用例，真实调用本工作站
GPU、非 CPU mock：①`K=1024/H=30` 单次 `solve()` 延迟 `<5ms`、零 NaN/Inf、
严格遵守每个推力器自己的 `thruster_max` 边界，且两个独立初始化、相同种子
的 `BaselineMppiController` 对相同输入给出可复现输出；②目标深度上移/
目标朝向偏航±90°的方向性代价梯度合理性检验（多次 receding-horizon 迭代
而非单次 MPPI 更新，避免被采样噪声掩盖真实梯度）；③`StateBaselineMppi`
250Hz ZOH 驱动假质点平面"虚拟物理"对 8s 模拟窗口的闭环位置跟踪收敛性，
同 `test_fsm_states.cpp` 的测试范式）全部通过，加上既有全部套件共 55/55
（9 个 gtest 可执行文件）零回归；lint 结果与既有基线完全一致（仅剩全仓库
预先存在的缺版权头/cppcheck 对 `.h` 误判为 C 代码等 Phase 2.14 起已记录的
失败，新增文件未引入任何新增 lint 回归）。真实 MuJoCo 闭环（`mujoco.
launch.py headless:=true use_rviz:=false estimation_mode:=ground_truth`，
经 `ros2 topic pub` 驱动）确认：切换到 `BASELINE_MPPI`（`command:=7`）不
崩溃，`/rov/controller_state` 正确发布 `current_state=7`、
`state_name=state_baseline_mppi`，推力器指令始终有限且在 `±40N` 范围内，
`/rov/runtime_diagnostics` 显示 `outer_loop_hz≈25.0`、
`outer_planner_solve_ms≈0.17~0.3ms`（远低于 `40ms` 外环预算）、
`timing_healthy=true`；进程通过 `SIGTERM` 干净退出，无残留进程。

## Phase 4.2（2026-10-20）：两阶段贴壁序列 + 壁面水面以上延伸 + 阵风循环 + 实时推力滑条 + 贴壁目标可视化 + 扰动力缩放修复

### 动机

既有 `ADHERE_PID`/`ADHERE_SMC`/`BASELINE_MPPI` 三个贴壁 FSM 状态进入时直接从
ROV 出生点开始逼近/贴壁，缺少一个明确的"先定深再贴壁"安全序列；水箱/壁面扩容
到 20m×20m×10m 后暴露了若干可视化/交互层面的副作用（阵风只播放一次、鼠标扰动
力随场景放大失真）；MuJoCo Viewer 的推力器滑条只能手动注入，无法反映控制器
真实输出，也缺少贴壁目标点的直观可视化。本阶段集中解决这六项问题。

### 两阶段贴壁序列（`DEPTH_ALIGN` → `WALL_ADHERE`）

新增 `ModelParams::AdhereStageParams`（`adhere_stage`，由 `robot_control.yaml`
扁平键 `adhere_target_depth_m`/`adhere_target_y`/`adhere_standoff_x`/
`adhere_depth_tol`/`adhere_vz_tol` 驱动，默认 `1.0m`/`0.0m`/`0.35m`/`0.05m`/
`0.05m`）——刻意作为三个贴壁算法共享的单一配置块，而非复制进
`adhere_pid.*`/`adhere_smc.*`/`baseline_mppi.*` 各自私有的参数块，因为"先统一
定深、再各自贴壁"正是本阶段要表达的架构意图。`FSMState.h` 新增虚函数
`adhereStageName()`（默认返回空字符串），`StateAdherePid`/`StateAdhereSmc`/
`StateBaselineMppi` 各自新增 `bool depth_aligned_`（sticky，`enter()` 重置为
`false`，一旦 `|z-Z_target|<=depth_tol` 且 `|v_z|<=vz_tol` 即永久置真，不会
倒退）并据此分支覆写 `adhereStageName()`。

`StateAdherePid`/`StateAdhereSmc`：`!depth_aligned_` 时每拍强制把 Y/Z 目标
覆写为 `adhere_stage.target_y`/`-adhere_stage.target_depth_m`，X 目标覆写为
`x_wall + adhere_stage.standoff_x`（安全站位，不触碰壁面）；`depth_aligned_`
之后才允许进入既有的 APPROACH（X 以 `approach_speed` 匀速逼近 `x_wall`）→
ADHERED（粘滞贴壁判定 `sum_f>=dock_force_thresh && x<=x_wall+x_dock_eps`）
两段逻辑，`docked_` 判定本身也新增 `depth_aligned_ &&` 前置条件，避免 ROV
仍在远离壁面的定深阶段时就误触发贴壁判定。顺带修复 `ModelParams::
AdherePidGains` 里 `kp_pos`/`ki_pos`/`kd_vel` 的 Z 分量此前被误置为 `0.0`
（文档注释写的是"Y/Z/姿态"三者都应被 `SixDofPidController` 主动调节，但
实际数组第三分量恒为零，使 Z 轴从未被真正镇定过——此前因为 Z 目标与起始值
总是相等而未被任何既有 gtest/闭环验证暴露），改为与 Y 分量一致的
`120.0/10.0/60.0`，否则 `DEPTH_ALIGN` 阶段的 Z 收敛要求无法被满足。

`StateBaselineMppi`：没有独立的 `docked_` 状态机——`!depth_aligned_` 时同样
把 Y/Z/X 目标覆写为站位点，`depth_aligned_` 之后把 X 目标直接改为
`BaselineMppiController::xWall()`（新增的只读访问器，暴露
`BaselineMppiConfig::x_wall`），剩下的逼近/压墙完全交给 CUDA 滚动代价里既有
的墙面接触惩罚项 + `preload_fx` 完成，不改动任何核函数。

`ControllerState.msg` 新增 `string adhere_stage` 字段（非贴壁状态为空
字符串），由 `AdhereController::update()` 在 25Hz 发布拍从
`current_state_->adhereStageName()` 填入，供下游（`adhere_mujoco_hardware`
的嵌入式 Viewer 叠加层/绿色目标球、RViz2 Marker）消费。

`test_adhere_wall_states.cpp`/`test_baseline_mppi.cpp` 相应更新：原先断言
"Y/Z 保持在 `enter()` 锁存的起始值"的两个 `ADHERE_PID`/`ADHERE_SMC` 收敛性
用例改为断言收敛到共享的 `adhere_stage.target_y`/`-target_depth_m`，并新增
`EXPECT_TRUE(state.isDepthAligned())`；`BASELINE_MPPI` 的闭环跟踪用例原先
驱动一个任意 `MODE_FULL_POSE` 设定点（`[0.2,0.15,-2.3]`，在两阶段序列下会被
无条件覆盖，测试会失去意义），改为驱动真实的两阶段收敛到壁面站位，并给这个
测试新增的 `FakePlant` 补上与 `test_adhere_wall_states.cpp` 同款的单边弹簧
贴壁接触模型（原来没有任何壁面物理，驱动目标穿过 `x_wall=0` 后会一直向负向
滑行而不停止）。

### 壁面延伸至水面以上 `1.5m`

`mjcf/rov_wall.xml` 的 `wall` 刚体 `pos`/`wall_geom` `size` 从
`"-0.01 0 -5.0"`/`"0.01 10.0 5.0"`（`Z∈[-10,0]m`）改为
`"-0.01 0 -4.25"`/`"0.01 10.0 5.75"`（`Z∈[-10,+1.5]m`，总高 `11.5m`），
`water_boundary_marker` 的局部 Z 同步从 `5.0` 改为 `4.25` 以继续标记世界
`Z=0.0m`；`adhere_mujoco_node.cpp` 的 RViz2 壁面 Marker 同步更新
`pose.position.z=-4.25`/`scale.z=11.5`。水面（`Z=0.0m`）与水箱底
（`Z=-10.0m`）不变。

### `12.0s` 阵风循环重放

`AdhereMujocoHardware::applyHydrodynamics()`/`write()` 两处原先都用未取模的
相对造浪时钟 `τ=max(0,t_sim-t_trigger)` 查询 `StationKeepingGustPlugin`，
使两段半正弦阵风在 `τ∈[3.0,4.2]s`/`[7.0,8.2]s` 播放一次后、`τ` 超出
`12.0s` 名义周期便再无阵风。两处改为用 `τ_mod=fmod(τ,12.0)` 查询，使阵风
模式在模式 `2` 保持激活期间永久循环（`τ∈[15.0,16.2]s`/`[19.0,20.2]s` 等
同理重放）；`MujocoVisBridge` 新增 `wave_tau_mod`，叠加层/遥测同时展示总
`τ` 与循环相位 `τ_mod`。

### 鼠标扰动力/箭头缩放修复

20m×20m×10m 水箱使 MuJoCo 自动估算的 `model_->stat.meansize`/`extent` 相对
`~0.2-0.6m` 的 ROV 壳体放大约 10 倍，而 `mjv_applyPerturbForce()`/
`mjv_updateScene()` 的扰动弹簧刚度与力箭头粗细/长度都按 `meansize` 缩放，
使轻微 `Ctrl`+拖拽即产生巨大扰动力与失真箭头。`AdhereMujocoHardware::
configure()` 中 `mj_loadXML` 后把 `model_->stat.meansize` 钳回 `0.25`
（匹配 ROV 尺度），`vis.scale.forcewidth`/`vis.map.force` 相应缩小为
`0.015f`/`0.04f`；`applyHydrodynamics()` 中额外对 `MujocoVisBridge::
xfrc_perturb` 读出的力/力矩范数做硬钳位（`<=40.0N`/`<=8.0N*m`），作为不依赖
GUI 侧视觉缩放公式细节的权威最后防线。

### `MujocoViewer` 实时推力滑条 + `Manual Override`

右侧边栏 "Thrusters (Manual)" 分组新增 `Manual Override` 复选框（默认关闭，
位于 8 个滑条上方）。关闭时，`renderFrame()` 每帧用 `d_vis_->ctrl[i]` 覆写
`thruster_slider_[i]`（滑条/数值框变成控制器实际输出的只读实时镜像），并把
`MujocoVisBridge::manual_ctrl` 强制清零；勾选后恢复原始行为（拖动滑条把值
写入 `manual_ctrl`，与上层控制器指令叠加），`Zero All` 按钮不受影响。

### 贴壁目标位姿遥测 + 绿色目标球可视化

`adhere_mujoco_node.cpp` 新增对 `/rov/controller_state` 的订阅（新增
`adhere_control_msgs` 消息级依赖，不引入任何 MuJoCo/CUDA/`ros2_control`
反向耦合），解析 `target_pose`/`current_state`/`adhere_stage`
并同时写入：(1) 节点本地原子量，驱动 `/rov/environment_markers` 新增的
绿色 `SPHERE` Marker（仅在 `adhere_stage` 非空时发布，位于世界
`(0, Y_target, Z_target)`）；(2) `MujocoVisBridge` 新增的
`fsm_current_state_code`/`adhere_stage_code`/`adhere_target_{x,y,z,yaw_deg}`
原子量，供嵌入式 `MujocoViewer` 消费。`MujocoViewer::drawOverlayTelemetry()`
叠加层新增 `Target Pos (X Y Z Yaw)` 与 `FSM State / Adhere Stage` 两行；
`renderFrame()` 新增同款绿色目标球场景几何（`mjGEOM_SPHERE`，半径
`0.035m`，RGBA `{0.1,1.0,0.2,0.9}`），同样只在贴壁状态激活时渲染。

### 验证结果

`export PATH=/usr/bin:/usr/local/cuda/bin:$PATH && colcon build --symlink-
install`（5 个包）零警告零错误。`colcon test`：全部 11 个 gtest 可执行文件
共 55/55 用例通过（含本阶段更新的 `test_adhere_wall_states`/
`test_baseline_mppi`），零回归；lint 结果与既有基线完全一致（仅剩全仓库
预先存在的缺版权头/`package.xml` `test_depend` 顺序等 Phase 2.14 起已记录的
失败，新增/修改文件未引入任何新增 lint 回归）。真实 MuJoCo 闭环（`ros2
launch simplified_v1_description mujoco.launch.py headless:=true
use_rviz:=false estimation_mode:=ground_truth`，经 `ros2 topic pub/echo`
驱动）确认：切换到 `ADHERE_PID`（`command:=5`）后 `/rov/controller_state`
依次报告 `adhere_stage=DEPTH_ALIGN`→`WALL_ADHERE`，`target_pose.position.z`
收敛到 `-1.0`，4 个 `contact_forces` 收敛到约 `9.55N`（均正、`kern_margin≈
0.9999986`）；切换到 `ADHERE_SMC`（`command:=6`）、`BASELINE_MPPI`
（`command:=7`）均不崩溃，`adhere_stage` 正确报告 `WALL_ADHERE`（ROV 已由
前序状态带到贴壁位置附近），`/rov/runtime_diagnostics` 显示
`outer_planner_solve_ms≈0.29~0.45ms`、`timing_healthy=true`；进程通过
`SIGTERM` 干净退出，无残留进程。

## Phase 4.3（2026-10-20）：子终端自动关闭 + FSM 状态名统一为 `FUNCTION_ALGORITHM_CATEGORY` + 通用目标位姿可视化 + 三个 `ADHERE_*` 基线调参

### 动机

Phase 4.2 完成两阶段贴壁序列后仍遗留三个问题：`fsm_console_node`/`state_cmd_
console_node` 弹出的子终端在主 launch 退出后不会自动关闭；8 个 FSM 状态名沿用
不一致的旧命名（`PASSIVE`/`ADHERE_PID`等），未遵循任何统一规则；绿色目标球/
`Target Pos` 叠加层遥测硬编码仅对三个 `ADHERE_*` 状态生效。本阶段集中解决
这三类问题，并顺带把三个 `ADHERE_*` 贴壁基线的触墙/保压参数调得更稳。

### 子终端生命周期：`--wait` + `OnProcessExit` 级联关闭 + 信号处理 + 看门狗线程

`adhere_keyboard_input/launch/teleop_gcs.launch.py` 的 `gnome-terminal` 回退
分支补上 `--wait`（`fsm_console_node.cpp::spawnStateConsole()` 内部早已这样
做，launch 文件这边之前漏了）。`simplified_v1_description/launch/mujoco.
launch.py` 新增 `RegisterEventHandler(OnProcessExit(target_action=
adhere_mujoco_node, on_exit=[EmitEvent(event=Shutdown())]))`——关闭嵌入式
MuJoCo GLFW 窗口导致 `adhere_mujoco_node` 退出时，`robot_state_publisher`/
`rviz2`/整个 `teleop_gcs.launch.py` 子树（`joy_node`/`xbox_teleop_node`/
`fsm_console_node`）会级联关闭。`fsm_console_node.cpp`/`state_cmd_console_
node.cpp` 都新增 `SIGINT`/`SIGTERM`/`SIGHUP` 信号处理器 + 一个每 `100ms`
轮询 `rclcpp::ok()` 的看门狗 `std::thread`（阻塞在 `std::getline(std::cin,
...)` 上时也能强制 `std::exit(0)`），并在 `fsm_console_node` fork 出的
`state_cmd_console_node` 子进程与 `state_cmd_console_node` 自身都加上
`prctl(PR_SET_PDEATHSIG, SIGTERM)`，作为既有 `setsid()`+`killpg()` 之外的
第二道防线（覆盖父进程被 `SIGKILL` 的场景）。

### FSM 状态名统一为 `FUNCTION_ALGORITHM_CATEGORY`（数值 ID 保持不变）

`CLAUDE.md` 第 7 节新增强制命名规则：每个 FSM 状态名（`FsmCommand`/
`FSMStateName` 枚举标识符及其小写 `state_name_string`）必须遵循
`功能_算法_类别` 三段式。8 个状态的数值 ID（`1`~`8`）保持不变，仅重命名
字符串：`PASSIVE→IDLE_PASSIVE_NONE`、`DEPTH_HOLD→HOLD_PID_DEPTH`、
`OPEN_LOOP→TELEOP_OPENLOOP_6DOF`、`PID_HOLD→HOLD_PID_6DOF`、
`ADHERE_PID→ADHERE_PID_WBC`、`ADHERE_SMC→ADHERE_SMC_WBC`、
`BASELINE_MPPI→ADHERE_MPPI_BASELINE`、`KAMR_MPPI→ADHERE_MPPI_KAMR`（最后
一个仍是未实现的占位状态，仅重命名消息常量/显示表）。改动覆盖
`FsmCommand.msg`/`ControllerState.msg` 常量与注释、`enumClass.h`、
`adhere_controller.cpp`/`FSMState.h` 的 int↔enum 映射、7 个 `FSM/StateXxx.
cpp` 构造函数、`adhere_keyboard_input` 全部 4 个节点、`mujoco_viewer.cpp`
的 `kFsmStateNames` 显示数组、`robot_control.yaml` 注释、相关 README 与
`test_adhere_wall_states.cpp`/`test_baseline_mppi.cpp` 的引用。保留了
`ADHERE_SMC_WBC`（用户确认：不随任务原文字面方案一起把 SMC 基线从 FSM 中
删除），仍维持 8 个状态、ID `1`~`8` 不变。

### 通用目标位姿可视化：`target_valid` 取代仅限 `ADHERE_*` 的硬编码门控

`ControllerState.msg` 新增 `bool target_valid`；`FSMState` 新增虚函数
`targetValid()`（默认 `false`），`HOLD_PID_DEPTH`/`HOLD_PID_6DOF`/
`ADHERE_PID_WBC`/`ADHERE_SMC_WBC`/`ADHERE_MPPI_BASELINE` 均覆写为 `true`
(`IDLE_PASSIVE_NONE`/`TELEOP_OPENLOOP_6DOF` 维持默认 `false`)。
`StateDepthHold::targetPose()` 不再固定返回 `[0,0,target_z_]`，改为每拍
缓存 ROV 的实际 X/Y（`last_actual_x_`/`last_actual_y_`），与 `target_z_`
一起返回，使绿色目标球落在 ROV 自身竖直轴线上而不是世界原点。
`MujocoVisBridge` 把 Phase 4.2 的 `adhere_target_{x,y,z,yaw_deg}`
原子量改名为通用的 `target_{x,y,z,yaw_deg}`（不再仅限贴壁状态）并新增
`target_valid`；`adhere_mujoco_node.cpp` 的 `/rov/controller_state` 回调与
`MujocoViewer::drawOverlayTelemetry()` 的叠加层/绿色球渲染逻辑都改为按
`target_valid` 门控（而非之前的 `adhere_stage` 非空判断），且按 FSM 类别
区分文案与球心 X 坐标：`IDLE_PASSIVE_NONE`/`TELEOP_OPENLOOP_6DOF` 显示
"None"；`HOLD_PID_DEPTH` 只显示 Z/深度；`HOLD_PID_6DOF` 显示完整
`[X,Y,Z,Yaw]`；三个 `ADHERE_*` 状态沿用 Phase 4.2 的视觉设计，球心 X 始终
钳到墙面 `0.0`（不随站位/逼近阶段的真实 X 漂移）。

### 三个 `ADHERE_*` 基线调参：软着陆阻尼 + 杠杆力矩前馈 + 提高保压目标

`ModelParams::AdhereStageParams` 新增共享字段 `approach_vx_damp`
(`adhere_approach_vx_damp`，默认 `80.0 N/(m/s)`)：`StateAdherePid`/
`StateAdhereSmc` 在 `depth_aligned_ && !docked_` 的 APPROACH 阶段，对命令
`Fx` 额外叠加 `-approach_vx_damp*max(0,v_x,body)` 项，使 ROV 在触墙前减速
而不是以恒定 `approach_speed` 直接撞墙反弹；`ADHERE_MPPI_BASELINE` 没有
独立的 APPROACH 状态机，改为把 `baseline_mppi.wall_b`（滚动代价里的触墙
阻尼）从 `200.0` 提高到 `500.0`，让 CUDA 滚动本身更偏好柔和逼近。
`StateAdherePid::run()`/`StateAdhereSmc::run()` 都在分配前对最终 wrench
追加杠杆力矩前馈 `T_y += 0.1*F_z, T_z -= 0.1*F_y`（对应 CLAUDE.md 第 5.2
节文档化的 `k_ff=0.1m`，补偿 `thr_T5..8` 在机体 `X=-0.10m` 处产生的寄生
倾覆力矩；`ADHERE_MPPI_BASELINE` 的 CUDA 滚动动力学模型在 Phase 4.1 已自带
等效项，未重复添加）。三个基线的目标保压力都调高，使稳态 4 垫压力落在
`~28~31N`/垫区间：`adhere_pid.f_des` `100.0→120.0`、`adhere_smc.
wall_preload_fx` `30.0→120.0`、`baseline_mppi.preload_fx` `30.0→120.0`。

### 验证结果

`export PATH=/usr/bin:/usr/local/cuda/bin:$PATH && colcon build --symlink-
install`（5 个包）零警告零错误。`colcon test`：全部 11 个 gtest 可执行文件
（含 `test_adhere_wall_states`/`test_baseline_mppi` 的重命名引用更新）共
563 个用例里全部 gtest 用例 0 失败，零回归；lint 失败（copyright/cpplint/
uncrustify 等）与既有基线完全一致，未引入任何新增 lint 回归。真实 MuJoCo
闭环烟雾测试本阶段未能完成：`ros2 launch simplified_v1_description
mujoco.launch.py headless:=true use_rviz:=false` 在 `adhere_controller`
`on_configure()`/`on_activate()` 阶段挂起超过 `90s`——经 `git stash` 复现
确认这是 Phase 4.2 既有代码库上已经存在的问题（与本阶段改动无关，怀疑是
本工作站当次会话的 CUDA/GPU 上下文初始化异常缓慢或卡死，需要下一阶段单独
排查），因此本阶段的终端自动关闭/`target_valid`/贴壁调参改动只经过了
gtest 套件验证，未经真实 MuJoCo 闭环烟雾测试复核，这点需要在下一次会话
优先补上。

## Phase 4.5（2026-10-20）：Ctrl+C 孤儿进程根因修复 + 三阶段贴壁定位序列 + 墙面 QR 码实时重定向 + Y 向防摩擦卡死修复

### 动机

本阶段收到的任务报告两个问题：① `ADHERE_PID_WBC`/`ADHERE_SMC_WBC`/`ADHERE_MPPI_BASELINE`
贴壁后会被卡在触墙瞬间的 Y 坐标，而不是收敛到目标 Y；② `Ctrl+C` 无法干净关闭所有衍生进程。
任务文本本身使用了 Phase 3.8 之前的旧 FSM 编号（`ADHERE_MPPI_BASELINE=6`），且假设 Y 卡死是
`target_y` 被硬编码为 `0.0` 导致——排查确认这两点都不对：当前代码 `ADHERE_PID_WBC=5,
ADHERE_SMC_WBC=6, ADHERE_MPPI_BASELINE=7`（Phase 3.8 起生效），且 `target_y`/`target_z`
早已正确贯穿三个贴壁状态，从未硬编码为 `0`。用户确认：① Ctrl+C 问题仍在真实复现（`ps aux`
显示多个存活 2 天以上的孤儿 `xbox_teleop_node`/`joy_node`），需要真正修复而非仅复核 Phase 4.3
既有方案；② `ADHERE_SMC_WBC` 应与另外两个贴壁基线同等对待；③ 墙面目标实时重定向改用全新
专属话题/消息（而非复用 `/rov/setpoint_cmd`），作为对 CLAUDE.md 第 3.4 节"4 话题正交契约"
的一次刻意、文档化的修订。

### Ctrl+C 孤儿进程根因与修复

现场复现（`ros2 launch adhere_keyboard_input teleop_gcs.launch.py` 后 `kill -INT`）确认：
`joy_node`/`xbox_teleop_node`/`fsm_console` 的 `ExecuteProcess` 包装器都能被 `ros2 launch`
的 SIGINT 传播正常终止，但 `gnome-terminal --wait` 派生出的整条进程树（`gnome-terminal.real`、
`ros2 run fsm_console_node`、`fsm_console_node` 本体）全部存活，被重新挂到 `systemd --user`
下——根因是 `gnome-terminal --wait` 并不把真正执行的命令作为 `ros2 launch` 自身进程树的
直接子进程，而是通过 D-Bus 激活共享的 `gnome-terminal-server` 守护进程来派生，`ros2 launch`
的 SIGINT/`Shutdown` 事件传播完全触及不到那条独立的进程树。Phase 4.3 为 `fsm_console_node`/
`state_cmd_console_node` 添加的信号处理器/看门狗线程/`prctl(PR_SET_PDEATHSIG)` 只在这些
进程**直接**收到信号时才生效，对这个根因完全无效。

修复：`teleop_gcs.launch.py` 新增一个 `OnShutdown` 事件处理器，执行
`pkill -TERM -f fsm_console_node`/`state_cmd_console_node`（`pkill -f` 按完整命令行匹配，
无论真实父进程是谁都能命中）。关键实现细节（现场验证发现）：这个清理动作**必须**是同步的
`OpaqueFunction`（内部调用阻塞的 `subprocess.call`），不能用 `ExecuteProcess`——实测确认
ROS 2 Foxy 的 `LaunchService._shutdown(force_sync=True)`（Ctrl-C 路径）同步发出 `Shutdown`
事件后立即着手拆除 asyncio 事件循环，一个在 `on_shutdown` 回调里才入队的 `ExecuteProcess`
根本没有机会被调度/启动（用一个仅含 `LogInfo`+`ExecuteProcess` 的最小复现脚本验证：
`LogInfo` 的日志正常打出，但 `ExecuteProcess` 从未产生任何输出），而 `OpaqueFunction` 里的
阻塞调用能在关闭流程继续之前完整跑完。修复后验证：全新 `teleop_gcs.launch.py` 启动 + 发送
`SIGINT`，`joy_node`/`xbox_teleop_node`/`gnome-terminal`/`fsm_console_node` 全部在 6 秒观察
窗口内干净退出，`ps aux` 零残留，无论作为独立 `ros2 launch` 运行还是被 `mujoco.launch.py`
通过 `IncludeLaunchDescription` 内含运行均生效。

排查过程中额外确认（而非引入）：会话开始时 `ps aux` 已有数个存活 1-6 天的孤儿
`xbox_teleop_node`/`joy_node`/`robot_state_publisher`（本阶段开工前已手动清理），且
`mujoco.launch.py` 的 `adhere_controller` `on_configure()` 挂起问题**仍然**复现（3 次独立
复现，均卡在打印 `"Configuring controller 'adhere_controller'"` 之后，70 秒以上无进展）——
确认排除了 Phase 4.3 猜测的 CUDA/GPU 上下文初始化原因（挂起点在 `on_activate()` 之前，
`BaselineMppiController::init()` 根本还未被调用），也排除了内存里记录的
`switch_controller()`-早于-`update()`-线程-启动经典死锁（`adhere_mujoco_node.cpp` 的
`load_controller`/`configure_controller`/`switch_controller` 调用顺序已经是正确的）。
真实根因本阶段未能在时间盒内定位——**遗留给下一阶段单独排查**（下一步可尝试：本工作站
`ptrace_scope=1` 导致 `gdb -p <pid>` 无 root 权限不可用，需要 `sudo`，或改用启动时就
attach 的 `strace -f`/`/proc/<pid>/task/*/stack`）。这直接导致本阶段的三阶段贴壁序列/
实时重定向改动未能跑通真实 MuJoCo 闭环烟雾测试，只经过 gtest 套件验证，与 Phase 4.3 同样
的诚实披露标准。

### Y 向卡死 bug 的真实根因（不是硬编码）

逐状态排查确认 `target_y_`/`target_z_` 早已正确贯穿 `StateAdherePid`/`StateAdhereSmc`/
`StateBaselineMppi`。真实根因：Phase 4.3 把贴壁压力目标 `f_des`/`wall_preload_fx`/
`preload_fx` 从 `30N`/`100N` 统一提高到 `120N` 后，库仑摩擦力随之增大，而：① `StateAdherePid`
的 `AdherePidController` 是纯 Fx 力环，Y 向完全依赖 `SixDofPidController` 的 Y-PID 单独对抗
摩擦力，原 `ki_pos.y=10.0` 在新摩擦量级下权限不足；② `SixDofSmcController` 的滑模面原本
**完全没有积分项**（只有比例 `lambda` + 开关 `k_switch`），静止卡死时速度误差 `e_nu≈0`，
开关项一旦饱和就不会随时间继续增大，没有任何机制能"攒力"压过静摩擦——这是三个基线里最
严重的一个；③ 旧的两阶段门限（`DEPTH_ALIGN`）只检查深度收敛就允许压墙，若此时 Y 还未收敛，
贴壁后就只能靠上述（权限不足的）Y 向控制对抗摩擦，而不是从一开始就避免压墙前 Y 未收敛的
情况。

### 三阶段贴壁定位序列：`STANDOFF_HOMING` → `SOFT_DOCKING` → `WALL_ADHERED`

重命名/扩展 Phase 4.2 的两阶段 `DEPTH_ALIGN`/`WALL_ADHERE`（数值 ID 不变，仅字符串/门限
扩展）。`ModelParams::AdhereStageParams` 新增 `y_tol`（`0.03m`）/`vel_tol`（`0.04m`，
`sqrt(vy^2+vz^2)` 平面速度门限）/`retarget_threshold`（`0.06m`，见下）；`depth_tol` 默认从
`0.05m` 收紧到 `0.03m`。三个贴壁状态的 `depth_aligned_` 粘滞标志改名为 `standoff_homed_`，
门限从"仅深度"扩展为"**Y 与深度与平面速度三者同时**收敛"——这是本 bug 的直接结构性修复：
压墙在结构上不可能发生在 Y 尚未收敛之前，与摩擦力大小/控制器权限无关。
`FSMState::adhereStageName()` 现报告三段字符串：`"STANDOFF_HOMING"`（`!standoff_homed_`）、
`"SOFT_DOCKING"`（`standoff_homed_ && !docked_`，仅 PID/SMC——MPPI 没有独立 `docked_` 子
状态机，只报告 `STANDOFF_HOMING`/`WALL_ADHERED` 两种）、`"WALL_ADHERED"`（`docked_`）；
`adhere_mujoco_hardware`（`adhere_mujoco_node.cpp`/`mujoco_viewer.cpp`/
`mujoco_vis_bridge.hpp`）相应把贴壁阶段编码从 `0..2` 扩展为 `0..3` 并更新字符串映射表。

`SixDofSmcController` 新增有界积分滑模项：滑模面 `s = e_nu + Lambda⊙e_eta + Ki⊙∫e_eta dt`
（新增 `Gains::ki`，默认全零不影响任何其它潜在用户，`adhere_smc.ki` 只在 Y/Z 设为非零），
`integral_e_eta_` 按固定设计常量 `kIntegralEtaClamp=2.0` 逐轴钳位防积分饱和；
`computeWrench()` 相应从 `const` 方法改为非 `const` 并新增 `dt` 参数，`resetIntegrator()`
纳入 `StateAdhereSmc::enter()`。`adhere_pid.ki_pos`/`adhere_smc.ki` 的 YAML 调参：
`ki_pos.y/z: 10.0→15.0/20.0`（非对称），`adhere_smc.ki: [0,15,20,0,0,0]`。

### 墙面 QR 码/AprilTag 预设与实时重定向：新话题 `/rov/adhere_target_cmd`

新增 `adhere_control_msgs/msg/AdhereTargetCommand.msg`（第 6 个正交话题，`adhere_control_msgs`
的"4 话题契约"经 Phase 3.6 扩到 5、本阶段扩到 6，CLAUDE.md 第 9 节同步记录这一刻意修订）：
`command_type`（`1=TAG_PRESET, 2=CUSTOM_Y, 3=CUSTOM_DEPTH, 4=CUSTOM_Y_DEPTH, 5=USE_CURRENT`）
+ `tag_id` + `target_y`/`target_depth_m`。`ModelParams` 新增
`std::vector<AdhereWallTag> adhere_wall_tags`（4 个默认预设，由 `robot_control.yaml` 的并行
扁平数组 `adhere_wall_tags_y`/`adhere_wall_tags_depth_m` 驱动，ROS 参数没有结构体数组类型）。

`CtrlInterfaces` 新增 `AdhereTargetState adhere_target_`（与既有 `OuterLoopCommand` 同款
无锁 `std::atomic` 桥接模式）：`target_y`/`target_depth_m`/`valid`/`changed_pending`，加上
`use_current_requested`/`command_seq` 两个专门处理 `USE_CURRENT` 的字段——`USE_CURRENT`
需要读 `ActiveState`，而 `ActiveState` 按 Phase 3.5 的地面真值隔离纪律只能在 500Hz 线程上
安全读取，因此 `AdhereController`'s 新 `/rov/adhere_target_cmd` 订阅回调（ROS 执行器线程）
只对 `TAG_PRESET`/`CUSTOM_*`（纯查表/纯读消息字段，无需跨线程状态）立即解析，`USE_CURRENT`
则仅记录请求+递增 `command_seq`，真正的 `ActiveState` 读取延后到 `AdhereController::
update()` 的 500Hz 线程本体执行。`on_activate()` 用 `ModelParams::adhere_stage` 的初值
种一次 `adhere_target_`，此后行为由该实时桥接唯一决定。三个贴壁 `FSMState::run()` 均改为
读取 `ctrl_interfaces_.adhere_target_` 而非静态的 `model_params_.adhere_stage.target_y/
target_depth_m`；每拍若检测到 `changed_pending`，立即把 `standoff_homed_`/`docked_` 清零
并消费该标志，强制重新进入 `STANDOFF_HOMING`，门限用 `adhere_stage.retarget_threshold`
（`0.06m`）。

`state_cmd_console_node` 在既有 `mode:=5/6/7` 的 `z/y/x/roll/pitch/yaw/all/reset/h` 语法
之外新增 `tag <0..3>`/`target y <val>`/`target d <depth>`/`target y <val> d <depth>`/
`target here` 命令，发布到新话题。刻意用 `tag`/`target` 前缀而非裸的 `y`/`d`——裸
`y <val>`/`z <val>` 在贴壁后仍保留既有"沿墙滑动设定点"功能（经由 `/rov/setpoint_cmd` 的
`MODE_FULL_POSE`），与"改写贴壁定位目标"是两个不同的既有/新增功能，用不同命令前缀避免语义
冲突。手柄摇杆微调目标的功能本阶段按计划推迟（非阻塞项）。

### gtest 覆盖

`test_adhere_wall_states.cpp`/`test_baseline_mppi.cpp` 均新增两类测试（PID/SMC/MPPI 三个
状态各一份）：①`...HomesLateralBeforeDocking.../...HomesLateralBeforeTargetingWallPlane...`——
从 `Y=0.75m` 大偏置起步，断言 `standoff_homed_`/`docked_` 首次变 `true` 的那一拍 Y 已经在
容差内（而不是"最终"收敛），是本 bug 的直接回归测试；②`...ReHomesOnLiveTargetChange`——贴壁
后直接写入 `ci.adhere_target_` 模拟一次 `tag 2` 到达，断言立即退出贴壁态并最终收敛到新目标。
两个文件里所有直接构造 `CtrlInterfaces`/`ModelParams` 的既有测试都补上了 `adhere_target_`
的显式种子赋值（此前隐式依赖 `ModelParams::adhere_stage` 默认值与 `AdhereTargetState` 的
编译期默认值恰好相同这一偶然事实）。

### 验证结果

`export PATH=/usr/bin:/usr/local/cuda/bin:$PATH && colcon build --symlink-install`（5 个包）
零警告零错误。`colcon test`：全部 11 个 gtest 可执行文件（含本阶段新增的 4 个用例）共 88 个
gtest 用例 0 失败，零回归；lint 失败（copyright/cppcheck/cpplint/uncrustify，共 572 项测试
中 370 项失败）逐一核对确认与既有基线完全一致（例如 copyright 的 56 项失败覆盖
`adhere_controller` 包内**全部** `.h`/`.cpp` 文件，包括本阶段完全未触碰的 `StatePassive.cpp`/
`test_thruster_allocator.cpp` 等——确认是 Phase 2.14 起已记录的全仓库缺版权头基线，未引入
任何新增 lint 回归）。真实 MuJoCo 闭环烟雾测试：如上文所述，受 `on_configure()` 挂起阻塞
未能完成，仅 `teleop_gcs.launch.py` 独立 Ctrl+C 清理验证通过（见上文）。

## Phase 5.0 Stage 0（2026-10-22）：`on_configure()` 挂起复现失败——结论为"当前不可复现"，非凭空判定修复

本阶段（框架"去补丁化"整合里程碑）的硬阻塞前置任务：诊断并修复 Phase 4.3/4.5 两次记录、
三次复现的 `adhere_controller::on_configure()` 挂起（日志卡在 `"Configuring controller
'adhere_controller'"` 后永不返回）。

**诊断方法**：在 `adhere_controller.cpp::on_configure()` 的每个子步骤（订阅创建/发布者创建/
`auto_declare` 速率参数/`ModelParams::loadFromNode()`/`BaselineMppiConfig::loadFromNode()`/
`MultiRateDecimator` 构造/返回）和 `adhere_mujoco_node.cpp` 的 `load_controller`→
`configure_controller`→`switch_controller`→`physics_thread` 全链路临时插入 `RCLCPP_ERROR`
级别探针日志（`ptrace_scope=1` 且无免密 `sudo` 导致 `gdb -p <pid>`/`strace -f` 均无法附加到
`ros2 launch` 派生的非直系子进程，遂改用日志探针而非调试器）。

**结果**：三种启动方式（`headless:=true use_rviz:=false launch_teleop:=false`；带 GLFW
窗口但无 RViz2/teleop；`mujoco.launch.py` 全部默认参数，RViz2+GLFW+teleop 终端全部开启）
**均未复现挂起**——`on_configure()` 全部子步骤在 <5ms 内顺序完成并返回 `SUCCESS`，
`configure_controller()`/`switch_controller()` 均即时返回 `OK`，`physics_thread` 以稳定
500Hz 持续运行（实测 tick 间隔 1.0 ± 0.01s / 500 ticks），`/rov/controller_state` 以
25Hz 发布真实 `sensor_fusion` 估计位姿（`state_idle_passive_none`，非零 `estimated_twist_body`
证明物理/估计器真实运行），`ros2 topic list` 确认全部 6 个话题 + `/joint_states`/`/tf` 均已
发布。探针日志随后已还原（`git checkout --`），未在仓库中保留任何临时调试代码。

**结论**：Phase 4.3/4.5 记录的挂起在本次三次独立尝试（跨 headless/窗口/RViz2 三种配置）中
均不可复现，原因未能定位（可能是本次之前某次未记录的依赖/环境变化间接修复，也可能与两次历史
复现时的某个尚未识别的会话态/首次运行条件相关）——**本阶段不编造一个"修复"**，按
`CLAUDE.md` 誠实标准记录为"当前状态：未复现，非确认修复"。经用户确认后，Stage 0 按"已解除
阻塞"处理，Stage 1 起的真实 MuJoCo 闭环验证据此判定为可执行（若未来某次真实复现，应在那次
立即重新打开本节诊断，而非假设已被这次的"未复现"永久排除）。

## Phase 5.0 Stage 1（2026-10-22）：删除 `HOLD_PID_DEPTH`、FSM ID 压缩至 1-7、删除设定终端、删除死抽象

本阶段 6 项全部是**删除/压缩**，不新增任何抽象或功能，净代码行数下降（`244` 行新增 /
`532` 行删除，净 `-288` 行；删除 5 个文件：`StateDepthHold.{h,cpp}`、
`state_cmd_console_node.cpp`、`algorithms/wbc/WbcAllocatorBase.h`、
`algorithms/mppi/MppiPlannerBase.h`）。

1. **删除 `HOLD_PID_DEPTH`**（与 `HOLD_PID_6DOF` 冗余）：移除 `FSM/StateDepthHold.{h,cpp}`、
   `ModelParams::DepthHoldGains`/`depth_hold.*` 加载与 YAML 键、`enumClass.h`/
   `adhere_controller.{cpp,hpp}` 的枚举与映射条目、`SixDofPidController.h` 的"仅深度"模式
   引用、`mujoco_viewer.cpp`/`xbox_teleop_node.cpp`（X 按键解绑）/`fsm_console_node.cpp` 的
   引用、`test_fsm_states.cpp`/`test_sensor_fusion_and_teleop.cpp` 的对应 gtest 用例，以及
   `ActiveState::rpy[3]`（唯一消费者就是该状态的小角度阻尼律；验证零其它读者后，连同
   `RovStateEstimator::rpy()` 这个仅为写它而存在的方法一并删除）。
2. **FSM `FsmCommand` ID 由历史的 `1-8` 压缩为 `1-7`**：
   `1=IDLE_PASSIVE_NONE, 2=TELEOP_OPENLOOP_6DOF, 3=HOLD_PID_6DOF, 4=ADHERE_PID_WBC,
   5=ADHERE_SMC_WBC, 6=ADHERE_MPPI_BASELINE, 7=ADHERE_MPPI_KAMR`——与删除 1 同一次改动完成，
   更新全部 4 个 msg、`adhere_controller.cpp`/`FSMState.h` 的映射表、`mujoco_viewer.cpp`
   的状态名数组与 Target Pos 显示分支（原 `fsm_code==2`"仅深度"分支随 `HOLD_PID_DEPTH`
   一起删除，`ADHERE_*` 判断区间由 `5..7` 平移为 `4..6`）、`adhere_mujoco_node.cpp` 的
   `is_adhere_state` 判断区间、全部 README 与 `CLAUDE.md` 第 7.5/9 节（第 7.5 节新增一次性
   豁免说明：ID 重新编号规则本身保持不变，但本次历史性压缩是用户确认的一次性例外，不作为
   未来惯例）。真实 MuJoCo 闭环验证：`ros2 topic pub /rov/fsm_command ... "{command: 3}"`
   确认切入的是新编号下的 `HOLD_PID_6DOF`（`current_state: 3`），而非旧编号。
3. **删除专属设定终端自动弹出机制**（手柄为主要交互手段，控制器自身位姿只是估计值，专属终端
   的"所见即所得"精度是假精度）：`fsm_console_node.cpp` 删除 `spawnStateConsole`/
   `killChildConsole`/`fork`/`setsid`/`killpg`/`prctl` 整套子终端生命周期管理代码与驱动它的
   "状态变化即弹窗"订阅逻辑分支，只保留 FSM 切换 + `/rov/controller_state`/
   `/rov/runtime_diagnostics` 状态读出；**完整删除 `state_cmd_console_node.cpp`** 及其
   `CMakeLists.txt`/`package.xml` 注册。手动数值设定点（深度/位姿/贴壁目标）改为直接
   `ros2 topic pub /rov/setpoint_cmd`/`/rov/adhere_target_cmd`。重新评估后**保留**
   `teleop_gcs.launch.py` 的 `OnShutdown`+`pkill` 孤儿进程清理机制（但删除其中
   `state_cmd_console_node` 的 `pkill` 行）——`fsm_console_node` 自身依然是通过
   `gnome-terminal --wait` 启动的，Phase 4.5 记录的 `gnome-terminal-server` 子进程脱离
   `ros2 launch` 进程树的根因对它本身依然成立，只是不再有它派生的嵌套子终端需要额外清理。
4. **删除两个零子类的死抽象**：`algorithms/wbc/WbcAllocatorBase.h`、
   `algorithms/mppi/MppiPlannerBase.h`（Phase 3.7 新增的纯数据契约占位头文件，
   `BaselineMppiController` 从未实现过它们）连同已空的 `wbc/`/`mppi/` 子目录一并删除。
   `KAMR-MPPI` 真正落地时会连同其具体实现一起重新引入接口，不再提前预留骨架。
5. **删除全部 7 处冗余的 `checkChange()` override**（`StatePassive`/`StateOpenLoop`/
   `StatePidHold`/`StateAdherePid`/`StateAdhereSmc`/`StateBaselineMppi`，均逐字重复基类
   `FSMState::checkChange() { return checkFsmCommandChange(); }`）：基类方法改为非 `virtual`
   （既然没有任何子类需要不同行为，`virtual` 本身就是多余的扩展点）。
6. **删除 `ActiveState::rpy[3]`**：已在第 1 项中随 `HOLD_PID_DEPTH` 一并处理，`quat_w[4]` 保留
   作为唯一姿态表示。

### 验证结果

`export PATH=/usr/bin:/usr/local/cuda/bin:$PATH && colcon build --symlink-install`（5 个包）
零警告零错误。`colcon test`：全部 gtest 可执行文件共 92 个用例、0 失败、零回归（lint 失败
为已记录的全仓库基线，未引入新增 lint 回归）。**真实 MuJoCo 闭环验证**（Stage 0 解除阻塞后
本阶段首次）：`headless:=true`/带 GLFW 窗口/`use_rviz:=true`+`launch_teleop:=true` 完整默认
参数三种配置下 `ros2 launch simplified_v1_description mujoco.launch.py` 均正常启动、
`on_configure`/`on_activate` 正常完成、500Hz 物理循环稳定运行，`ros2 topic pub
/rov/fsm_command ... "{command: 3}"` 确认新 ID 编号下的状态切换生效，Ctrl+C/`pkill`
后无孤儿进程残留。

## Phase 5.0 Stage 2（2026-10-22）：统一 FSM 层——单一注册表、共享贴壁基类、增益结构体合并、单一无锁桥接原语

本阶段是本次整合里程碑里最大的一块去重：三个 `ADHERE_*` `FSMState::run()`（约 600 行）此前
~90% 逐字复制粘贴（实时墙面目标桥接、`STANDOFF_HOMING` 收敛门、`SOFT_DOCKING` 对接门、
`MODE_*` 设定点边沿检测、手柄摇杆→位姿积分+倾角钳位、三段式 `p_d_world_` 覆盖、杠杆臂力矩
前馈）；FSM 状态的 int↔枚举↔字符串↔实例映射此前分散在 4 处独立手工维护的位置；
`SixDofPidController`/`SixDofSmcController`/`AdherePidController` 的增益结构体此前在
`ModelParams` 里被逐字段镜像复制、靠手写 `namespace{}` 拷贝函数同步；`OuterLoopCommand`/
`AdhereTargetState` 两个跨线程无锁桥接各自独立发明了 `valid`/`seq` 记账惯用法。

1. **单一 FSM 注册表**（新增 `common/FsmRegistry.h`）：`kFsmRegistry`（`{id, FSMStateName,
   name_string}` 的 `constexpr` 数组）是 `FsmCommand` 数字 ID↔`FSMStateName`↔显示字符串的唯一
   权威映射，`fsmIdToStateName()`/`fsmStateNameToInt()`/`fsmStateNameToString()` 均从它派生。
   `FSMState::checkFsmCommandChange()`（原 7 分支 `switch`）与 `adhere_controller.cpp` 的
   `fsmStateNameToInt()`（原 6 分支 `switch`，已删除独立定义，改用头文件里的同名函数）均改为
   调用这组函数。`AdhereController::FSMStateList`（6 个具名字段 + 6 行 `make_shared`）与
   `getNextState()` 的 6 分支 `switch` 合并为单个 `std::vector<std::pair<FSMStateName,
   std::shared_ptr<FSMState>>> fsm_states_`（在 `on_activate()` 里按声明顺序构造，支持不同
   构造函数签名）+ 一次线性扫描查找（只在 FSM 切换时调用，非 500Hz 热路径）。新增一个状态
   现在只需：`enumClass.h` 加一行枚举、`FsmRegistry.h` 加一行表项、`fsm_states_` 加一行
   `make_shared`——而不是过去的 6+ 处编辑点。`mujoco_viewer.cpp`/`fsm_console_node.cpp` 等其它
   包因架构上不能 `#include` `adhere_controller` 头文件，继续维护各自独立的名称数组（已在
   注释中说明）。
2. **`common/PoseSetpointIntegrator.h`**（ROS 无关、头文件 only、可独立 gtest，与
   `OutputSafety.h` 同风格）：把 `StatePidHold`/`StateAdherePid`/`StateAdhereSmc`/
   `StateBaselineMppi` 逐字复制的 `MODE_*` 设定点边沿检测 + 手柄摇杆位姿积分 + 倾角钳位整体
   抽出为一个类，拥有 `p_d_world_`/`q_d_` 并提供 `initPose()`/`applyDiscreteSetpoint()`/
   `integrateTeleop()`。四个调用方唯一的真实行为差异——摇杆前进/后退是否驱动 `p_d_world_.x()`
   ——参数化为 `integrateTeleop()`的 `drive_surge` 布尔量（经验证：三个 `ADHERE_*` 状态无论
   `drive_surge` 取何值，X 分量都会被后续的三段式覆盖逻辑无条件重写，因此传 `false` 与原
   `StateBaselineMppi` 的 `true` 在行为上完全等价，不是回归）。
3. **`FSM/StateAdhereBase.{h,cpp}`**：新的抽象基类承载三个 `ADHERE_*` 状态共享的整条贴壁
   序列——实时墙面目标重定向桥接、`standoff_homed_`/`docked_` 黏性门、三段式 `p_d_world_`
   覆盖、`approach_vx_damp` 软着陆项、杠杆臂力矩前馈、`b_pinv_` 分配写出。具体子类只需实现
   `computeAdhesionWrench(pos_world, q_est, vel_body, omega_body, sum_f, dt, wrench_out)`
   （返回 `true` 表示 `wrench_out` 已填好、交给基类做前馈+分配；`StateBaselineMppi` 返回
   `false`，因为它是严格 ZOH 直接写 `pending_thruster_cmd`，没有 wrench/分配器这一步）
   + 若干静态配置访问器（`xWall()`/`hasDockPhase()`/`approachSpeed()`/`dockForceThresh()`/
   `xDockEps()`/`maxTeleopVel()`/`maxTiltDeg()`）。三个 `.cpp` 从约 600 行降到
   `StateAdherePid.cpp` 82 行 + `StateAdhereSmc.cpp` 59 行 + `StateBaselineMppi.cpp` 48 行
   （共 189 行）+ 新增共享的 `StateAdhereBase.cpp` 145 行——净大幅下降且运行时行为验证不变
   （见下文测试结果）。
4. **合并镜像增益结构体**：`ModelParams::PidHoldGains`/`AdherePidGains` 现在直接公有继承
   `algorithms::pid::SixDofPidController::Gains`（`AdherePidGains` 额外继承
   `algorithms::pid::AdherePidController::Gains`），`AdhereSmcGains` 直接公有继承
   `algorithms::smc::SixDofSmcController::Gains`——不同的默认值通过派生类自己的构造函数体赋值
   （基类字段默认值改为全零/占位值，不再是这 3 个算法各自的历史默认值）。为避免
   `ModelParams.h`→算法头文件→`FossenDynamics.h`→`ModelParams.h` 的头文件循环，
   `SixDofPidController.h`/`SixDofSmcController.h` 把原先 `#include
   "adhere_controller/dynamics/FossenDynamics.h"` 改为前向声明 `class FossenDynamics;`（只持有
   `const FossenDynamics &` 引用成员，不需要完整定义），两个对应 `.cpp` 补上完整 include。
   `StatePidHold.cpp`/`StateAdherePid.cpp`/`StateAdhereSmc.cpp` 里原先的 4 个
   `namespace{}` 手写拷贝函数（`pidHoldGainsToSixDof`/`adherePidGainsToSixDof`/
   `adherePidGainsToForcePid`/`adhereSmcGainsToController`）全部删除，构造函数直接传
   `model_params.pid_hold`/`model_params.adhere_pid`/`model_params.adhere_smc`（通过基类
   子对象隐式转换）。`ModelParams.cpp` 的 YAML 加载代码完全不需要改动（公有继承保留字段名，
   赋值路径不变）。
5. **单一无锁桥接原语**（新增 `common/AtomicBridge.h::AtomicBridgeBase{valid, seq}`）：
   `CtrlInterfaces.h` 的 `OuterLoopCommand`/`AdhereTargetState` 改为公有继承它，不再各自声明
   独立的 `valid`/`command_seq` 字段——`valid`/`seq` 字段名通过公有继承保持不变，因此绝大多数
   既有 `ctrl_interfaces_.outer_loop_cmd_.valid.load()` 风格调用点不受影响；仅
   `AdhereTargetState` 原先的 `command_seq` 改名为继承来的 `seq`（`adhere_controller.cpp`
   的 3 处调用点相应更新）。`use_current_requested`/`changed_pending`/`target_y`/
   `target_depth_m` 等语义不同的字段保留在各自结构体内，未被强行塞进共享基类。

### 验证结果

`export PATH=/usr/bin:/usr/local/cuda/bin:$PATH && colcon build --symlink-install`（5 个包）
零警告零错误。`colcon test`：全部 9 个 gtest 可执行文件 0 失败、零回归（lint 失败为已记录的
全仓库基线，未引入新增 lint 回归）。本阶段暂未重复 Stage 1 已验证过的真实 MuJoCo 闭环烟雾
测试（FSM 行为学语义未变，仅内部结构重排，gtest 覆盖的正是这些语义：三段式贴壁收敛顺序、
实时重定向、ZOH 直通、PID/SMC/MPPI 三个基线的完整闭环 Markdown 对比），计划在 Stage 3-8
全部完成后做一次完整的真实闭环回归，而非每个子阶段各做一次。

## Phase 5.0 Stage 3（2026-10-22）：MultiRateDecimator 泛化为具名通道注册表

`common/MultiRateDecimator.h` 从固定的 `{inner_due, outer_due}` 二元组改为通用具名通道注册表：
`addChannel(name, decimation)` 返回稳定的 channel id，`tick()` 按注册顺序返回每个通道本拍是否
触发的 `vector<bool>`，`periodSeconds(id)`/`decimation(id)`/`name(id)` 按 id 查询。
`AdhereController::on_configure()` 现在注册 4 个通道——`inner`(`inner_decimation_`)、
`outer`(`outer_decimation_`)、`diag`(固定 `cm_update_rate_/10` -> 10Hz)、
`estimator`(decimation=1，仅为显式记录其逐拍节奏，门控条件仍是
`estimation_mode=="sensor_fusion"` 不变)——取代原来分离的 `decimator_`
（仅 inner/outer）+ 独立的 `diag_decimation_`/`diag_tick_count_` 计数器机制。
`update()` 一次 `decimator_->tick()` 后用 4 个 `channel_` 成员索引取值，删除原有的
`diag_tick_count_ % diag_decimation_` 手写取模。Depth/DVL 传感器速率诊断的
`have_last_*_diag_` 边沿检测**保留**（而非替换成另一个注册通道）——头文件新增注释说明原因：
`sensor_depth_hz`/`sensor_dvl_hz` 存在的意义就是**验证**硬件真的在按标称速率降频，一个"构造上
必然正确"的 decimator 通道无法验证这件事，只有基于真实采样值变化的边沿检测才能。

`test_multirate_decimator.cpp` 全部 4 个既有用例迁移到新 API，并新增
`ArbitraryNamedChannelsFireAtTheirOwnRate`（同时验证 inner/outer/diag/estimator 4 个通道各自的
触发次数与周期），验证本次泛化没有改变任何既有速率的实际行为。

### 验证结果

`export PATH=/usr/bin:/usr/local/cuda/bin:$PATH && colcon build --symlink-install
--packages-select adhere_controller` 零警告零错误。`colcon test`：全部 9 个 gtest 可执行文件
共 52 个用例、0 失败、零回归（lint 失败为已记录的全仓库基线，未引入新增 lint 回归）。

## Phase 5.0 Stage 4（2026-10-22）：Sim/Real 噪声参数拆分 + 环境标记单一数据源

**仿真侧传感器噪声 -> MJCF**：`adhere_mujoco_hardware.cpp` 匿名命名空间里的
`kGyroNoiseStd`/`kGyroBiasRwStd`/`kAccelNoiseStd`/`kAttNoiseStd`/`kDepthNoiseStd`/
`kDvlVelNoiseStd`/`kDvlRangeNoiseStd`/`kLoadcellNoiseStd`/`kDepthDecimation`/`kDvlDecimation`
十个 `constexpr` 移入新的 `SensorNoiseParams` 结构体（`include/adhere_mujoco_hardware/
sensor_noise_params.hpp` + `src/sensor_noise_params.cpp`），`applyMjModelOverrides()` 严格镜像
`HydroParams::applyMjModelOverrides()` 的 `<custom><numeric name="sensor_noise_*"/>` 查找模式。
`rov_wall.xml` 新增 `<custom>` 块，用当前默认值显式声明全部 10 个 `sensor_noise_*` numeric——
不是无操作占位，而是让这个模型文件成为这些参数的唯一权威来源（修改 XML 即生效，零 C++ 重
编译）。`AdhereMujocoHardware::configure()` 在两次 `HydroParams::applyMjModelOverrides()` 调用
之后紧接着调用 `SensorNoiseParams::applyMjModelOverrides(model_, sensor_noise_)`，同一生命周期
点。

**估计器假定增益 -> `robot_control.yaml`**：`RovStateEstimator.h` 的 8 个
`static constexpr`（`kAttGain`/`kDepthPosGain`/`kDepthVelGain`/`kDvlVelGain`/`kDvlRangeGain`/
`kContactForceFilterGain`/`kContactVelGain`/`kContactPosGain`）移入新的
`ModelParams::EstimatorGains` 结构体，通过 `RovStateEstimator` 的新显式构造函数注入（默认参数
保留原编译期默认值，任何未显式传参的既有调用方——包括全部既有 gtest——行为不变）。
`ModelParams::loadFromNode()` 新增 `estimator.*` 8 个 YAML 键的加载，`AdhereController::
on_configure()` 在 `model_params_ = ModelParams::loadFromNode(...)` 之后立即
`estimator_ = RovStateEstimator(model_params_.estimator)`，同一生命周期点重建估计器。

**环境标记单一数据源**：`adhere_mujoco_node.cpp` 曾手工硬编码 `wall`/`water_surface`/
`floor` 三个 RViz `/rov/environment_markers` 的位置与尺寸（如 `wall.pose.position.z=-4.25`,
`wall.scale=(0.02,20.0,11.5)`），必须每次改 `rov_wall.xml` 几何时手动同步。现在
`AdhereMujocoHardware::configure()` 在解析推进器/传感器站点的同一处，新增一段从 `model_` 自身
`wall_geom`/`floor_geom`/`water_surface` 三个 geom 读出世界坐标（`geomWorldPos()`：
`body_pos + body_quat * geom_pos`，三者均是 worldbody 的直接子节点，无需走完整的运动学链）与
尺寸（`geom_size` 是半长，RViz CUBE 的 `scale` 是全长，故 ×2）写入 `MujocoVisBridge` 的新
write-once 字段（`wall_pos_world`/`wall_size_full`/`floor_z`/`floor_extent_xy`/
`water_surface_z`/`water_surface_extent_xy`，与既有 `pad_site_pos_body` 同一惯用法）；
`adhere_mujoco_node.cpp` 的 `TelemetrySnapshot` 新增对应字段，marker 发布代码删除全部硬编码
字面量，改读 `snap.*`。真实 MuJoCo 运行验证：`/rov/environment_markers` 发布的
`wall.pose.position=(-0.01,0,-4.25)`/`wall.scale=(0.02,20.0,11.5)` 与删除前的硬编码值逐位
一致，证明推导路径正确。

### 验证结果

`export PATH=/usr/bin:/usr/local/cuda/bin:$PATH && colcon build --symlink-install`（全部 5 个
包）零警告零错误。`colcon test`：`adhere_controller` 52 个 gtest 用例、`adhere_mujoco_hardware`
35 个 gtest 用例，共 0 失败、零回归（lint 失败为已记录的全仓库基线）。真实 MuJoCo 闭环验证：
`headless:=true use_rviz:=false launch_teleop:=false` 启动后 `/rov/environment_markers`
发布内容与 Stage 4 改动前的硬编码值逐位一致，Ctrl+C 后无孤儿进程残留。

## Phase 5.0 Stage 5（2026-10-22）：拆分 `adhere_keyboard_input` + 新增 `perception/` 分类

`adhere_keyboard_input`（遥操作终端 + 可观测性/离线评估混在一个包里）按关注点拆分为
**两个**零 MuJoCo/CUDA/ros2_control 依赖的独立包：

- `commands/adhere_teleop_gcs`：`fsm_console_node` + `xbox_teleop_node` +
  `launch/teleop_gcs.launch.py` + `config/xbox_teleop.yaml`（操作员输入/GCS）；
- `commands/adhere_monitor`：`rt_monitor_node` + `benchmark_evaluator_node`
  （可观测性/离线评估，`benchmark_evaluator_node` 仍是本包中唯一允许订阅
  `/rov/ground_truth_odom` 的节点）。

全部用 `git mv` 保留历史，旧包目录整体删除。`simplified_v1_description/launch/
mujoco.launch.py` 的 `get_package_share_directory("adhere_keyboard_input")` 与
`launch_teleop` 描述改为 `adhere_teleop_gcs`；`fsm_console_node.cpp` 里引用
`rt_monitor_node` 的提示文本随之改为 `ros2 run adhere_monitor rt_monitor_node`。
`CLAUDE.md` 第 6/9 节包树、`src/commands/README.md`、`src/interfaces/README.md`、
`src/interfaces/adhere_control_msgs/README.md`、`src/controllers/adhere_controller/
README.md` 的全部引用同步更新；两个新包各补一份完整 README.md（基于拆分前最后一次
已提交版本 + 当前源码重新核对撰写——本阶段 `git rm -rf` 旧包目录时，发现 Stage 1/2
对旧 `README.md` 的未提交编辑已随文件一起被删除且不可恢复，PROGRESS.md 的 Stage 1/2
记录条目本身完好无损，用于重新核对新 README 内容的准确性；新增的两份 README 已对照
当前 `fsm_console_node.cpp`/`xbox_teleop_node.cpp`/`rt_monitor_node.cpp`/
`benchmark_evaluator_node.cpp` 源码逐项核对，没有造成信息丢失，仅造成一次文档编辑
历史记录的中断）。

新增顶层分类 `perception/`（与 `hardwares/` 并列，为真实 `apriltag_ros` 边界镀一个
镜像位置），本阶段同时创建了其第一个软件包骨架 `adhere_apriltag_detector`（随即在
Stage 6 填入真实节点实现，未分两次提交）。

## Phase 5.0 Stage 6（2026-10-22）：AprilTag 两级贴壁目标（替代弹出式设定终端）

新增 `TagDetection.msg`（`adhere_control_msgs`，发布在 `/rov/tag_detection`）：
`station_id`/`target_y`/`target_depth_m`/`confidence`/`valid`，与
`/rov/adhere_target_cmd`（人工意图）严格区分——这是一条**测量**话题。
`AdhereController::on_configure()` 新增 `tag_detection_subscription_`，收到
`valid=true` 的检测后调用新抽出的 `applyResolvedAdhereTarget(y, depth_m)`
（把原来内嵌在 `adhere_target_command_subscription_` 回调里的
`retarget_threshold`/`changed_pending`/`last_adhere_target_*` 写入逻辑提炼成
一个共享私有方法，`/rov/adhere_target_cmd` 的 `TAG_PRESET`/`CUSTOM_*` 分支与
`/rov/tag_detection` 共用同一条路径）——两个话题在同一个 `ctrl_interfaces_.
adhere_target_` 桥接上"后写入者生效"，这就是任务要求的"测得标签优先于更早的预设，
直到操作员显式重新预设"契约：检测器只要 ROV 仍在站点范围内就持续重新发布，预设
不会悄悄滞留。

`perception/adhere_apriltag_detector`（新 `adhere_apriltag_detector_node`）是一个
**邻近触发的点发布器，不是视觉管线**：复用 `robot_control.yaml` 的
`adhere_wall_tags_y`/`adhere_wall_tags_depth_m` 站点表（通过自己的同名 ROS 参数，
默认值与之字面一致），订阅 `/rov/controller_state` 的**估计**位姿（绝不读地面
真值），按 `publish_rate_hz`（默认 `5.0`）定时查找最近站点，若欧氏距离
（Y-深度平面）`<= detection_range_m`（默认 `0.5m`）则发布该站点坐标 + 一个
`position_noise_std_m`（默认 `0.02m`）高斯偏移 + `confidence`，否则发布
`valid=false`（而非停止发布，便于判断检测器存活）。`mujoco.launch.py` 新增
`launch_apriltag_detector`（默认 `true`）启动该节点。`simplified_v1_description/
README.md` 新增真实贴纸规格一节（`tag36h11`，单标签 `80mm`，2x2 矩阵间距
`160mm`，矩阵外廓 `240x240mm`，背板 `300x300mm` 防水哑光覆膜，4 站点共 16 个 ID，
`station_id = min_id/4`，含 `apriltag_ros` `tag_bundles` 局部坐标表）——纯文档，
不涉及代码；仿真侧检测器本身不需要这套 2x2 几何，只用 4 个站点中心点。

### 验证结果

`export PATH=/usr/bin:/usr/local/cuda/bin:$PATH && colcon build --symlink-install`
（全部 7 个包，`adhere_control_msgs` 新增 `TagDetection.msg`）零警告零错误。
`colcon test`：`adhere_controller` 52 个 + `adhere_mujoco_hardware` 35 个共 87 个
gtest 用例、0 失败、零回归（lint 失败为已记录的全仓库基线）。

**真实 MuJoCo 闭环验证受阻，但已排除是本阶段改动引起的回归**：本阶段首次尝试
真实闭环烟雾测试时，`ros2 launch simplified_v1_description mujoco.launch.py` 在
`"Configuring controller 'adhere_controller'"` 日志后挂起不返回——与 Phase
4.3/4.5/Stage 0 三次记录过的同一个未解决挂起**症状完全一致**（Stage 0 当时
"三次独立尝试均未复现"，本阶段在同一会话的后续测试中又稳定复现了 5 次）。
为排除这是本阶段新增代码（`tag_detection_subscription_`/新增
`adhere_apriltag_detector` 节点）引起的回归，做了二分验证：①
`launch_apriltag_detector:=false`（完全不启动新节点）——挂起依旧；②临时注释掉
`tag_detection_subscription_` 的创建代码、单独重新编译 `adhere_controller`、
复测——挂起依旧，随后已还原代码并重新编译确认恢复。两项排除后确认挂起与 Stage 6
的改动无关，是 Stage 0 记录的同一个预先存在、间歇性、未定位根因的问题，在本次
长会话中间歇性复现（本阶段额外排除了一个可能的误导性假设：会话期间大量
`kill -9` 式终止 ROS 2 进程在 `/dev/shm` 残留了约 290 个 `fastrtps_*` 共享内存/
命名信号量文件，猜测可能导致 DDS 发现变慢卡死；清空 `/dev/shm` 后挂起依旧复现，
排除此假设）。按 `CLAUDE.md` 的誠实标准，本阶段的真实闭环验证继续被这个 Stage 0
就已记录为"未解决"的问题阻塞，**不代表 Stage 5/6 本身引入了新缺陷**——所有新增
逻辑均通过 gtest（现有 87 个用例全绿）与独立节点级验证覆盖：单独运行
`adhere_apriltag_detector_node`（不经过完整 `mujoco.launch.py`）确认其 5Hz
定时器正确触发、`/rov/controller_state` 订阅正确接线（`ros2 node info` 核实）、
无 `/rov/controller_state` 输入时正确持续发布 `valid=false`（临时插入
`RCLCPP_INFO` 探针确认 tick 行为后已移除，未保留在仓库中）。`on_configure()`
挂起的诊断/修复仍需要专门的下次会话时间（`CLAUDE.md` Stage 0 当时建议的
`strace -f`/`/proc/<pid>/task/*/stack` 等手段本次未再次尝试，不在本阶段时间盒内）。

## Phase 5.0 Stage 7（2026-10-22）：消息整理——拆分 `BenchmarkMetrics.msg`、修复陈旧注释、删除死 YAML 键

**从 `ControllerState.msg` 拆出 `BenchmarkMetrics.msg`**：`thrust_l2_norm`/
`thrust_power_proxy`/`thrust_slew_rate_l2`/`cumulative_energy_l2`/`cumulative_impulse_l1`/
`saturated_thruster_count`/`active_contact_pad_count`/`kern_margin` 共 8 个 Phase 3.8
效率/啁啾/安全遥测字段移入新的 `BenchmarkMetrics.msg`（+ `current_state`/`state_name`
使单条样本自描述），发布在 `/rov/benchmark_metrics`，与 `/rov/runtime_diagnostics` 共用
`AdhereController::update()` 里同一个 10Hz "diag" `MultiRateDecimator` 通道。
`ControllerState.msg` 恢复精简，只保留 FSM/位姿反馈。`kern_margin` 的计算（需要
`outer_due` 块里才有的 `a_contact_`/`u_cmd`）缓存进新成员
`AdhereController::latest_kern_margin_`，在 `diag_due` 块里随 `latest_metrics_`
一起打包进 `BenchmarkMetrics` 发布。重新接线三个消费者：`adhere_monitor` 的
`benchmark_evaluator_node`（新增 `/rov/benchmark_metrics` 订阅，缓存最新一帧，在
`/rov/controller_state` 回调里取用——两话题速率不同 10Hz vs 25Hz，用"取最近一帧"
而非强制同步）与 `rt_monitor_node`（效率/安全表格整表改订阅 `/rov/benchmark_metrics`）；
`adhere_teleop_gcs` 的 `fsm_console_node`（单行效率摘要同样改订阅新话题）。

**修复陈旧注释/死字段**：`ControllerState.msg` 的 `adhere_stage` 注释
`"DEPTH_ALIGN"/"WALL_ADHERE"` 更新为当前真实值 `"STANDOFF_HOMING"/"SOFT_DOCKING"/
"WALL_ADHERED"`；`SetpointCommand.msg`/`AdhereTargetCommand.msg` 的"published by
state_cmd_console_node"引用改为"`ros2 topic pub` 直接发布"（该终端已在 Stage 1 删除）；
`adhere_controller.cpp` 里残留的 `state_cmd_console_node` 注释引用一并清理。审计
`robot_control.yaml` 的死键：`adhere_pid.f_preload` 核实**仍被**
`AdherePidController::computeWrench()` 消费，**不**删除；`adhere_vz_tol`
核实除了加载进 `ModelParams::AdhereStageParams::vz_tol` 外**没有任何 FSM 状态读取
过这个字段**（`grep` 零匹配），确认是真死字段，连同其 YAML 键、`ModelParams.h` 字段、
`ModelParams.cpp` 加载代码一并删除；`depth_hold.*` 整块在 Stage 1 已经删除，本阶段
核实确认（零残留）。

**重新梳理话题契约**：`CLAUDE.md` 第 9 节的话题列表从 "7 ROS Topics" 更新为
"8 ROS Topics"，新增 `/rov/benchmark_metrics` 条目并把 `/rov/controller_state`
条目的字段摘要精简为实际剩余字段；`adhere_control_msgs/README.md` 的话题表格
同步扩充为全部 8 行（含本阶段新增的 `TagDetection.msg`/`BenchmarkMetrics.msg`
两行）；`adhere_controller/README.md` 的 `ControllerMetrics.h` 一节更新为指向新话题。

### 验证结果

`export PATH=/usr/bin:/usr/local/cuda/bin:$PATH && colcon build --symlink-install`
（全部 7 个包，`adhere_control_msgs` 新增 `BenchmarkMetrics.msg`）零警告零错误。
`colcon test`：`adhere_controller` 52 个 + `adhere_mujoco_hardware` 35 个共 87 个
gtest 用例、0 失败、零回归（lint 失败为已记录的全仓库基线）。真实 MuJoCo 闭环
验证：继续被 Stage 6 记录的同一个预先存在的 `on_configure()` 挂起问题阻塞（本阶段
纯消息/文档重构，未改动任何 on_configure()/on_activate() 逻辑，风险评估为与该挂起
无关）。

## Phase 5.0 Stage 8（2026-10-23）：最终验证、整合记分卡与文档收尾

### 整合记分卡

**三个 `ADHERE_*` `.cpp` 文件**（Stage 2，共享基类 `StateAdhereBase` 抽出后）：

| 文件 | 改动前（约） | 改动后 | 变化 |
|---|---|---|---|
| `StateAdherePid.cpp` | ~244 行 | 82 行 | 删除 162 行 |
| `StateAdhereSmc.cpp` | ~197 行 | 59 行 | 删除 138 行 |
| `StateBaselineMppi.cpp` | ~166 行 | 48 行 | 删除 118 行 |
| 新增共享 `StateAdhereBase.{h,cpp}` | — | 145 行（`.cpp`）+ 头文件 | 新增一次，三方共用 |
| **三文件合计（`git diff --stat`）** | — | — | **净 -454 行**（`48` 行新增 / `502` 行删除） |

**FSM 映射编辑点**：从 Stage 2 之前的 6+ 处独立维护位置（`checkFsmCommandChange()`
`switch`、`fsmStateNameToInt()` `switch`、`getNextState()` `switch`、`FSMStateList`
具名字段 + 7 行 `make_shared`、`mujoco_viewer.cpp`/键盘节点各自的名称数组）压缩为
`common/FsmRegistry.h` 一张 `constexpr` 表 + `fsm_states_` 一个 `vector`——新增一个
状态现在只需 2 行编辑（枚举 + 注册表），而非过去的 6+ 处。

**删除的文件**（Stage 1，共 5 个；Stage 5 的包拆分走 `git mv` 保留历史，不计入"删除"）：
`StateDepthHold.{h,cpp}`、`state_cmd_console_node.cpp`、
`algorithms/wbc/WbcAllocatorBase.h`、`algorithms/mppi/MppiPlannerBase.h`。

**删除的死 YAML 键**：`depth_hold.*` 整块（Stage 1，随 `HOLD_PID_DEPTH` 一起）、
`adhere_vz_tol`（Stage 7，核实零消费者后删除）。保留但核实非死键：
`adhere_pid.f_preload`（仍被 `AdherePidController::computeWrench()` 消费）。

**删除/统一的跨线程无锁桥接惯用法**：`OuterLoopCommand`/`AdhereTargetState` 从各自
独立发明 `valid`/`seq` 字段，统一成公有继承自新 `common/AtomicBridge.h::
AtomicBridgeBase`（Stage 2）。

**整体改动规模**（`git diff --cached --shortstat`，Phase 5.0 全部 8 个 Stage 合计，
`git mv` 保留历史的文件按"修改"而非"删除+新增"计入）：

```text
91 files changed, 2914 insertions(+), 2578 deletions(-)
```

净 `+336` 行——**不是负数**，这是预期且经过核实的结果，不是对整合目标的违反：
Stage 0-2（Part A 诊断出的 10 个发现点的实际删除/去重工作）本身净 `-288` 行
（Stage 1 的删除）+ 三个 `ADHERE_*` 文件净 `-454` 行（Stage 2 的共享基类抽取）等
纯减法改动，合计这部分远为负数；但 Stage 3-7 按任务书明确要求新增了若干**新能力**
而非去重——Stage 4 的 `SensorNoiseParams`/`EstimatorGains`（把硬编码常量变成可配置
参数，必然新增代码）、Stage 5 的 `perception/adhere_apriltag_detector` 全新包、
Stage 6 的 `TagDetection.msg` + 检测器节点 + 两级贴壁目标逻辑、Stage 7 的
`BenchmarkMetrics.msg` 拆分（新消息类型）——这些都是任务书 Part A 诊断之外、
Stage 3 起才新增的范围，净增代码是必然的，并不违反"删除类改动净行数下降"这一
验收标准（该标准在任务书原文中明确只约束 Stage 0-2 的 10 个发现点，Stage 3 起的
"Stage 3-8"标题本身也不再使用"Subtractions"这个词）。

**删除文件计数**：5 个（Stage 1）+ 0 个新删除（Stage 3-8 均为新增/拆分，`adhere_
keyboard_input` 的拆分通过 `git mv` 保留全部文件历史，严格意义上不是"删除"）。

### 最终真实 MuJoCo 闭环验证状态

如 Stage 6/7 record 所述，`ros2 launch simplified_v1_description mujoco.launch.py`
在本次长会话的后段持续复现 `on_configure()` 挂起（与 Phase 4.3/4.5/Stage 0 记录的
同一个预先存在、未定位根因的间歇性问题完全一致），经二分验证确认与 Stage 5/6/7
的任何改动无关（禁用新增的 `tag_detection_subscription_`/`adhere_apriltag_detector`
节点后挂起依旧；清空 `/dev/shm` 残留的 FastRTPS 共享内存/信号量文件后挂起依旧）。
按 `CLAUDE.md` 的誠实标准，本次 Phase 5.0 的"真实 MuJoCo 闭环烟雾测试"验收项**未能
完成**，与 Phase 4.3/4.5 当时的记录处于同一诚实状态——所有 Phase 5.0 Stage 0-7
的改动均已通过以下方式交叉验证：
- `colcon test`：全部 7 个包、87 个 gtest 用例、0 失败、零回归（全程各 Stage 独立
  验证，详见各自小节）；
- 独立节点级验证（不经过完整 `mujoco.launch.py`）：`adhere_apriltag_detector_node`
  单独运行确认 5Hz 定时器/发布逐帧正确（Stage 6）；`ros2 launch ... headless:=true`
  在 Stage 3/4 两次单独会话早段成功完整跑通（`/rov/environment_markers`
  逐位核对、FSM 状态切换验证），证明 `on_configure()` 挂起是**间歇性**而非
  **确定性**失效——不能排除某次干净的重新启动（新终端/重启工作站）就能再次跑通；
- 代码审查：所有新增/修改的 `on_configure()`/`on_activate()` 改动点逐一与挂起复现
  时的最小改动集二分比较，排除是本阶段引入的回归。

`on_configure()` 挂起的根因定位仍是**下一阶段的首要阻塞项**，建议下次会话一开始
就用 `strace -f`/`/proc/<pid>/task/*/stack`（`CLAUDE.md` Stage 0 当时记录的下一步
手段，本阶段因时间盒未重新尝试）或 `gdb`（需确认 `ptrace_scope` 设置）在挂起发生
的瞬间附加诊断。

### 本阶段文档收尾

`CLAUDE.md` 第 9 节话题列表更新为完整的 "8 ROS Topics"（新增
`/rov/benchmark_metrics` 条目，`/rov/controller_state` 条目精简为实际剩余字段）；
第 4 节（原始设计稿的"4-Topic"枚举）保留为原始设计快照，补充一条简短的"已被
第 9 节修订"指针注释（`FsmState.msg`/`adhere_keyboard_input`/
`state_cmd_console_node` 均已不存在，但原文本身不重写，遵循本文件一贯的
"Sec.9 是修订权威来源"惯例）；第 6 节包树已在 Stage 5 更新为含
`perception/`/两个拆分后的 `commands/` 子包；第 7.5 节的 FSM ID 一次性豁免说明
已在 Stage 1 写入。全部相关 `README.md`（`adhere_control_msgs`、
`adhere_controller`、`commands/README.md`、`interfaces/README.md`）均已在
Stage 5/7 同步更新完毕。
