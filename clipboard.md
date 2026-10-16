# Task: Comprehensive Code Audit and Hydrodynamic Enhancement Analysis for `adhere_mujoco_hardware`

Please perform a rigorous, line-by-line audit of all source and header files inside `src/hardwares/adhere_mujoco_hardware/` (including `adhere_mujoco_hardware.cpp/.hpp`, `fossen_hydrodynamics.cpp/.hpp`, `flow_environment_plugin.cpp/.hpp`, `mujoco_viewer.cpp/.hpp`, `adhere_mujoco_node.cpp`, launch files, config files, and tests)[span_1](start_span)[span_1](end_span).

Your objectives are:
1. **Code Audit & Bug Detection**: Check for thread-safety issues, race conditions on `MujocoVisBridge`, memory management, ROS 2 Foxy API compatibility, MuJoCo data/pointer safety, and numerical stability in the 500Hz physics loop[span_2](start_span)[span_2](end_span).
2. **Completeness & Feature Gaps**: Identify any missing architectural features, control hooks, or telemetry requirements.
3. **Hydrodynamic Simulation Accuracy**: Analyze both `kParametricLumped` and `kStonefishGeometryDriven` hydrodynamic modes in `fossen_hydrodynamics.cpp`[span_3](start_span)[span_3](end_span). Propose concrete mathematical or algorithmic enhancements to maximize simulation fidelity (e.g., higher-order fluid-structure interaction, frequency-dependent damping, vortex-induced effects).
4. **Output Format & Interaction Requirement**:
   - Provide your full code analysis, identified issues, and enhancement recommendations in **Simplified Chinese (中文)**.
   - At the end of your response, explicitly **ask the user** (in Chinese) which of the proposed recommendations they would like to modify or implement.
 

 # Fossen 2011 *Handbook of Marine Craft Hydrodynamics and Motion Control*
## Section 2.6 Kirchhoff’s Equations for a Triaxial Ellipsoid
半轴：$\(a,b,c\)$；椭球体积 $V=\tfrac43\pi abc$；流体密度 $\rho$。
$\kappa_1,\kappa_2,\kappa_3$ 为无量纲附加质量系数，来自含参数 $\lambda$ 的椭圆积分。

---
### Eq. (2.159)
$$
\kappa_i=\int_{0}^{\infty}\frac{abc}{2\sqrt{(a^2+\lambda)(b^2+\lambda)(c^2+\lambda)}}\frac{d\lambda}{r_i^2+\lambda},\quad i=1,2,3
$$
where
$$
r_1=a,\quad r_2=b,\quad r_3=c
$$

---
### Eq. (2.160) Translational added mass coefficients（平动附加质量）
$$
\begin{cases}
m_{A,11}=\rho V \dfrac{\kappa_1}{2-\kappa_1}\\[6pt]
m_{A,22}=\rho V \dfrac{\kappa_2}{2-\kappa_2}\\[6pt]
m_{A,33}=\rho V \dfrac{\kappa_3}{2-\kappa_3}
\end{cases}
$$

---
### Eq. (2.161) Rotational added inertia coefficients（旋转附加惯量）
$$
\begin{cases}
I_{A,11}=\dfrac{\rho V}{5}\dfrac{(b^2-c^2)^2(\kappa_3-\kappa_2)}{2(b^2-c^2)+(b^2+c^2)(\kappa_2-\kappa_3)}\\[8pt]
I_{A,22}=\dfrac{\rho V}{5}\dfrac{(a^2-c^2)^2(\kappa_1-\kappa_3)}{2(a^2-c^2)+(a^2+c^2)(\kappa_3-\kappa_1)}\\[8pt]
I_{A,33}=\dfrac{\rho V}{5}\dfrac{(a^2-b^2)^2(\kappa_2-\kappa_1)}{2(a^2-b^2)+(a^2+b^2)(\kappa_1-\kappa_2)}
\end{cases}
$$

---
### Eq. (2.162) Rigid-body inertia matrix + added inertia
刚体惯量 $\boldsymbol{M}_{RB}$，附加惯量 $\boldsymbol{M}_A$；总惯量
$$
\boldsymbol{M}=\boldsymbol{M}_{RB}+\boldsymbol{M}_A
$$
$$
\boldsymbol{M}_{RB}=
\begin{bmatrix}
m\boldsymbol{I}_{3\times3} & -m\boldsymbol{S}(\boldsymbol{r}_g)\\
m\boldsymbol{S}(\boldsymbol{r}_g) & \boldsymbol{I}_g
\end{bmatrix}
$$
$m$：刚体质量；$\boldsymbol{r}_g=[x_g,y_g,z_g]^T$ 重心在体坐标系；$\boldsymbol{S}(\cdot)$ 反对称矩阵；$\boldsymbol{I}_g$ 刚体转动惯量。

---
### Eq. (2.163) 三轴椭球附加惯量矩阵 $\boldsymbol{M}_A$ 的分块形式
$$
\boldsymbol{M}_A=
\begin{bmatrix}
\boldsymbol{A}_{11} & \boldsymbol{A}_{12}\\
\boldsymbol{A}_{21} & \boldsymbol{A}_{22}
\end{bmatrix}
$$
For a triaxial ellipsoid with origin at ellipsoid center: all off-diagonal blocks are zero：
$$
\boldsymbol{A}_{12}=\boldsymbol{A}_{21}=\boldsymbol{0}_{3\times3}
$$
$$
\boldsymbol{A}_{11}=\mathrm{diag}\big(m_{A,11},\;m_{A,22},\;m_{A,33}\big),\quad
\boldsymbol{A}_{22}=\mathrm{diag}\big(I_{A,11},\;I_{A,22},\;I_{A,33}\big)
$$

---
### Eq. (2.164) Full 6×6 added inertia matrix
$$
\boldsymbol{M}_A=
\begin{bmatrix}
m_{A,11} & 0 & 0 & 0 & 0 & 0\\
0 & m_{A,22} & 0 & 0 & 0 & 0\\
0 & 0 & m_{A,33} & 0 & 0 & 0\\
0 & 0 & 0 & I_{A,11} & 0 & 0\\
0 & 0 & 0 & 0 & I_{A,22} & 0\\
0 & 0 & 0 & 0 & 0 & I_{A,33}
\end{bmatrix}
$$

> 紧接着 Eq.(2.165) 就是四元数姿态微分方程，不再属于椭球附加质量。

如果你需要，我可以提供：
1. 长椭球（$\(b=c\)$）特例下 $\kappa_1,\kappa_2$ 的解析闭式（不用数值积分）；
2. Python 代码，直接输入 $\(a,b,c\),\rho$ 算出整套 $m_{A,ii},I_{A,ii}$。