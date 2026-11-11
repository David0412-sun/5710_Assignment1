# Assignment 1 课程核对记录

核对日期：2026-11-11。结论：现有 FK/IK 的主要数学实现符合 ROSE5710 课件中的坐标变换、Modified D-H 和有约束数值优化原理，也保留了本地历史模板要求的 IK 方法。需要处理的是 Windows 环境、报告用语与来源说明；没有根据课件改动三张 D-H 表或将 IK 换成其他求解器的理由。

## 核对依据和范围

课程文件位于本机 `C:\Users\MSI\Desktop\University_Study_26-27_Sem1\ROSE5710`：

| 文件 | 本次主要核对页 |
| --- | --- |
| `LS3 Robot Modelling Foundations.pdf` | 22–25：旋转逆变换、复合旋转和齐次变换 |
| `LS4 Forward Kinematics.pdf` | 20、23–28、38–40、52、54–57：D-H 约定、坐标系、矩阵和 FK |
| `LS4 Forward Kinematics (annotated).pdf` | 23、25–28：关节轴、公共法线和右手坐标系的图示与批注 |
| `LS5 Inverse Kinematics (annotated).pdf` | 30–35：误差最小化、关节限制、初值、梯度和 SLSQP 示例 |

这里的页码是从 1 开始的 PDF 页码，与对应页面右下角的课件页码一致。关键公式、坐标图及课件批注已通过渲染图片核对，不仅依靠文本提取。

作业的具体数值配置另与仓库历史提交 `7d701262308eb659426f619da35f6d2f6e9184c3` 中的两个 TODO 模板核对：

```powershell
git show 7d70126:assignments/assignment_1_kinematics/forward_kinematics.py
git show 7d70126:assignments/assignment_1_kinematics/inverse_kinematics.py
```

另已核对项目根目录的 `Assignment 1.pdf`：第 3–5 页图 2–4 的左右腿和头部原图、尺寸与 D-H 表要求，第 6 页的 IK 误差函数、中心差分、固定步长与终止条件，以及第 7 页的提交要求。报告使用图 2–4 的无损矢量裁剪，并保留原标注；来源与裁剪哈希记录在 `report/assets/assignment_original_source.json`。机器人的具体尺寸与关节方向同时对照作业原图、仓库 XML 和现有实现，课件提供通用建模方法。

简化图使用 `J_i` 表示关节、`ell_i` 表示刚性连杆，以免与原图关节标签 `L_i` 混淆。坐标轴沿用原图的 x 蓝、y 红、z 绿，并逐项列出所有 D-H 坐标系零位的三轴方向。关节物理正方向为 `sign_i * z_i`；腿部 frame 0 与基座仅原点重合，头部 frame 0/1 也仅原点重合。已验证各 D-H 原点位于对应的真实关节轴线上，而不强制等于 CAD joint anchor：最大轴方向误差 `2.20e-14`，原点到轴线最大距离 `4.05e-14 m`，末端位置最大误差 `4.11e-14 m`。

2026-11-12 补充标注核对：将简化图上的原点明确标为 `O_i`，并在作业原图上叠加 `O_i`、中间点 `P_i`、橙色 `a_(i-1)` 和紫色 `d_i`。每行采用 `O_(i-1) → P_i → O_i`，验证 `P_i=O_(i-1)+a_(i-1)x_(i-1)` 及 `O_i=P_i+d_i*z_i`。原图部分关节符号、基座轴与尺寸符号不满足同一个严格三维投影，因此原图叠加线用于说明选点和参数归属；正方向与有向距离由基座系中的坐标轴标注和数值表明确给出，不从纸面的左右、上下推断。特别是左腿 `d1=-0.0741 mm` 为所选 yaw/roll 交点相对基座的净高度；XML 的 yaw anchor 高度为 `45.9409 mm`，至 roll 轴的高度变化为 `-46.015 mm`，两者之和才为 `-0.0741 mm`。不能把原图圆柱中心、CAD anchor 和 D-H 原点当作同一点。

## 核对通过的数学和实现

| 项目 | 课件或模板依据 | 当前实现与结论 |
| --- | --- | --- |
| 单行 D-H 变换 | LS4 第 20、52 页；FK TODO 1 | `Rx(alpha) @ Tx(a) @ Rz(theta) @ Tz(d)` 与课件完全相同，属于 Modified D-H，不能替换成另一种 D-H 顺序。 |
| 参数下标与关节轴 | LS4 第 23、25、40、52 页 | 使用 `alpha_(i-1)`、`a_(i-1)`、`d_i`、`theta_i`；关节轴是 `z_i`。报告的下标与矩阵正确。 |
| 参数存储顺序 | LS4 第 18、43–45 页；FK 模板的行格式 | 课件印刷表格为 `[alpha, a, d, theta]`，代码为 `[alpha, a, theta_offset, d]`。代码解包与报告表格生成各自一致，已明确说明转换，不能仅交换表头而不转换数值。 |
| 坐标系选取 | LS4 第 25–28 页 | 相交轴可取零公共法线长度；平行轴的公共法线位置存在自由度。代码选择交点和合并平行轴方向的固定偏移是允许的，不能要求 D-H 原点一律等于 CAD body 原点。 |
| 关节符号和零位偏移 | FK TODO 3、5、7；LS4 第 40、44 页 | 使用 `theta_i = signs[i] * q[i] + theta_offset`，保留模型关节正方向与所选 D-H 轴之间的映射。左右腿不能只靠复制同一组符号完成镜像。 |
| 链式乘法与固定末端 | LS4 第 54、57 页；FK TODO 1–7 | 从 `T_base_0` 开始按关节顺序右乘；腿有 5 个关节行加 1 个固定 site 行，头部有 4 个关节行加 1 个固定 site 行。固定行不叠加关节变量。 |
| 世界系转基座系 | LS3 第 22–25 页 | `site_pose` 使用 `R_base.T @ (p_site - p_base)` 和 `R_base.T @ R_site`，等价于 `inv(T_world_base) @ T_world_site`。仅减平移而省略旋转会在基座旋转后出错；现有实现没有这个问题。 |
| IK 目标函数 | LS5 第 30–32 页；IK TODO 1 | 位置误差和相对旋转向量分别构成平方项，使用 `100 * ||e_p||^2 + ||e_R||^2`。姿态误差不是逐分量相减欧拉角。 |
| 初值与关节约束 | LS5 第 32、35 页；IK 模板 | 从当前链关节角开始，后续轨迹目标 warm start；更新后按 XML 限位逐关节裁剪。 |
| 数值梯度和更新 | IK TODO 2–3 | 中心差分 `(E(q+h e_i)-E(q-h e_i))/(2h)`、`h=1e-6`、固定 `alpha=0.1`、最多 5000 次迭代，均与历史模板一致。 |
| 终止与独立验收 | IK 模板 | 保留 `cost < 1e-8` 的停止条件，并最终读取 MuJoCo site 验收位置与姿态。非负平方项保证该 cost 条件对应 FK 模型误差分别小于 `1e-5 m` 和 `1e-4 rad`；这不意味着任何目标都必然收敛。 |
| 测试轨迹和初始姿态 | IK 历史模板的 `run` 及三个 trajectory 函数 | 左右脚 20 mm 轨迹、头部绕颈轴 0.1 rad 的可达弧线、初始关节角、0.19 m 基座高度、5 周期和每周期 101 相位均保留。headless 模式增加了无图形验证入口。 |

实现位置见 [forward_kinematics.py](assignments/assignment_1_kinematics/forward_kinematics.py) 和 [inverse_kinematics.py](assignments/assignment_1_kinematics/inverse_kinematics.py)。

## 需要澄清的课程和报告表述

1. **区分推导模型与数值求值。** LS4 第 55–57 页把代入关节值后进行矩阵循环累乘称为 numerical FK。报告已改为“解析建立 MDH 模型，再数值计算 FK”，并补上 LS4 引用。现有零位检查有显式解析表达式，因此保留 `Analytical zero-pose check`。
2. **区分课件原理与模板要求。** LS5 第 34–35 页使用 `scipy.optimize.minimize(..., method='SLSQP')` 作为示例，并未要求所有作业使用它。当前中心差分、固定步长及其数值参数来自历史 IK 模板。报告已分别引用课件与模板，说明参数来源，并保留模板算法。
3. **不要照抄课件的标题笔误。** LS4 第 38、39 页标题分别写成 `d_(i-1)` 和 `theta_(i-1)`，但本页图示、第 40 页汇总以及第 52 页变换公式一致使用 `d_i`、`theta_i`。现有作业用的是正确下标，无需修改。
4. **不要把课件简化的目标函数当作本题完整位姿定义。** LS5 第 30–33 页给出平方范数，第 34 页示例代码为简化的一维数组误差范数。原模板明确要求带位置尺度的平方误差以及 rotation vector。当前实现正确保留该要求；范数与平方范数即使具有相同的零误差目标，其梯度与固定步长迭代也不同。
5. **实验平台和版本必须对应实际记录。** 原有 `results/environment.json` 记录的是 Linux 实验。Windows 的运行结果应来自本机重新运行，并与 Python 版本、依赖版本和源文件指纹一起保存；报告不能只把 Linux 字段改成 Windows 来充当实测。

## Windows 适配事项

这些属于运行环境和复现流程，不是 D-H 数学错误。本次环境配置和报告构建修复由主任务单独执行，具体入口与完成后的运行说明见 [README.md](README.md)。

- 原说明以 `.venv/bin/python` 和 Bash 命令为主；Windows 应使用项目内 `.venv\Scripts\python.exe`，无需永久修改 PowerShell 执行策略。
- 本机最初可见的 Python 3.13/3.14 不适合直接安装本项目锁定的 NumPy 1.26.4 等依赖；使用独立 Python 3.11 环境，并保留已有系统 Python。
- 作业只需要 `assignments/assignment_1_kinematics/requirements.txt` 中的轻量依赖。根目录原训练工程的 `jax[cuda12]` 等依赖不应成为 FK/IK 在 Windows 上运行的前提。
- 报告构建还需可用的 TeX 工具链和可写的构建目录；MiKTeX、BibTeX 的实际执行结果应与数值仿真分别验证。

## 本机实测结果

已在 Windows、Python 3.11.15、NumPy 1.26.4、SciPy 1.14.1、MuJoCo 3.2.4 环境执行 `run_assignment.ps1 verify`，退出码为 0：

- 12 项回归测试通过：6 项运动学测试、6 项报告构建测试。
- FK 三条链各 100 个随机位姿，共 300 个；位置和姿态误差均低于 `1e-6 m`、`1e-5 rad`。
- IK 三条链各 5 周期 × 101 相位，共 1,515 个位姿；全部通过。
- MuJoCo 使用 Windows OpenGL 成功渲染实际机器人模型；本次数值验证使用 headless 模式。

| IK 链 | 最大位置误差 (mm) | 最大姿态误差 (rad) |
| --- | ---: | ---: |
| 左腿 | 0.009848 | 6.846e-5 |
| 右腿 | 0.009845 | 7.038e-5 |
| 头部 | 0.008393 | 6.773e-5 |
| 验收上限（严格小于） | 0.010000 | 1.000e-4 |

完整精度数据、样本、Windows 环境和源码指纹已同步更新到 `assignments/assignment_1_kinematics/results/`。这些是本机重跑的结果，不是将旧 Linux 记录重新标注。

报告改为普通课程作业格式：A4 单栏、11 pt 正文、简洁标题和页码，依次说明任务、FK、IK、验证结果与总结，保留 D-H 表、必要公式、实测图表和编号参考资料。使用本机 MiKTeX pdfLaTeX + BibTeX 编译，并在生成后逐页渲染检查。最终构建状态及输入文件指纹记录在 `output/latex/compile_manifest.json`；PDF、LaTeX 源码包和提交包一同更新。Windows 编译流程保留了 TeX 输出编码兼容修复。
