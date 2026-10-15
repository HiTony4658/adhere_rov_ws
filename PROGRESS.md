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
