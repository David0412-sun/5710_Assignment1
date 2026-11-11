# ROSE5710 Assignment 1: Kinematics

本项目完成 Open Duck Mini 双足机器人的左腿、右腿与头颈链运动学作业：Modified D-H 推导、正运动学（FK）、有限差分梯度下降逆运动学（IK），以及与 MuJoCo site 位姿的比较。

This assignment implements modified D-H forward kinematics and bounded, fixed-step gradient-descent inverse kinematics for both legs and the head. Verification compares both position and orientation with MuJoCo.

## 环境 / Setup

在本目录执行以下命令，使用 Python 3.11。作业所需的轻量 MuJoCo 环境不依赖 JAX 或训练环境。

当前工作区的 `.venv/bin/python` 已准备好，可直接运行下面的验证命令；新环境可按以下步骤安装。

Run these commands from this directory using Python 3.11. The kinematics environment does not require JAX or the training stack.

```bash
python3.11 -m venv .venv
.venv/bin/python -m pip install -r assignments/assignment_1_kinematics/requirements.txt
```

Windows 对应解释器路径为 `.venv\Scripts\python.exe`。

## D-H 约定 / Convention

每行变换为 `Rx(alpha) @ Tx(a) @ Rz(theta) @ Tz(d)`，其中 `theta = signs[i] * q[i] + theta_offset`；最后一行为不受关节角影响的固定 site 行。

The code stores rows as **`[alpha, a, theta_offset, d]`**, whereas the assignment's printed tables use **`[alpha, a, d, theta]`**. Translations are in metres and angles are in radians. The three tables and sign vectors are defined in `forward_kinematics.py` and listed in the report.

## 验证 / Run and compare

正运动学：每条链使用固定随机种子测试 100 个关节配置。位置误差要求小于 `1e-6 m`，姿态误差小于 `1e-5 rad`。

FK checks 100 seeded random configurations per chain:

```bash
.venv/bin/python assignments/assignment_1_kinematics/forward_kinematics.py
.venv/bin/python assignments/assignment_1_kinematics/forward_kinematics.py --output-json assignments/assignment_1_kinematics/results/fk_results.json
```

逆运动学：默认完整运行 5 个周期，每周期 101 个相位，跟踪左右脚和头部的原始目标轨迹。保留作业指定的中心差分 `h=1e-6`、固定步长 `alpha=0.1`、最多 5000 次迭代及 `cost < 1e-8` 停止条件。位置误差要求小于 `1e-5 m`，姿态误差小于 `1e-4 rad`。

IK runs the supplied trajectories for five cycles with 101 phases each. The graphical version opens the MuJoCo viewer; headless mode removes rendering and its delay while preserving the algorithm, targets and tolerances:

```bash
# Graphical viewer / 图形界面
.venv/bin/python assignments/assignment_1_kinematics/inverse_kinematics.py

# Full numerical comparison / 完整数值验证
.venv/bin/python assignments/assignment_1_kinematics/inverse_kinematics.py --headless --output-json assignments/assignment_1_kinematics/results/ik_results.json
```

macOS 图形查看器请先激活环境，然后用 `mjpython` 运行：

```bash
source .venv/bin/activate
mjpython assignments/assignment_1_kinematics/inverse_kinematics.py
```

`--cycles`、`--samples` 与 `--parts` 可用于局部调试；正式比较保留默认参数。完整运行并达到全部误差要求时，脚本返回退出码 0。

Use `--cycles`, `--samples` and `--parts` for smaller diagnostic runs. Keep the defaults for the full comparison. Successful complete comparisons exit with status 0.

## 测试与报告 / Tests and report

报告现采用真正的 LaTeX 编译流程，使用标准 `IEEEtran` A4 双栏论文格式，包含摘要、关键词、编号公式、三线表、交叉引用和 IEEE 参考文献。正文源码位于 `assignments/assignment_1_kinematics/report/assignment_1_report.tex`，可直接编辑。

The report is compiled from LaTeX using the standard IEEEtran conference class,
with numbered equations, cross-references, booktabs tables, a vector figure and
a BibTeX bibliography. The build retains compilation logs and checks the final
log for errors, unresolved references, missing glyphs and overfull boxes.

```bash
.venv/bin/python -m unittest assignments.assignment_1_kinematics.test_kinematics -v

# Optional report-generation dependencies / 可选报告依赖
.venv/bin/python -m pip install -r assignments/assignment_1_kinematics/requirements-report.txt
.venv/bin/python assignments/assignment_1_kinematics/build_report.py
```

编译需要 Tectonic，或带 `IEEEtran` 宏包的 TeX Live/MiKTeX。构建脚本会检测当前工作区可用的编译器；Tectonic 首次运行可能需要下载宏包和字体。也可使用 `--engine tectonic`、`--engine latexmk` 或 `--engine pdflatex` 指定引擎。完整源码编译说明见 [LaTeX 源码说明](assignments/assignment_1_kinematics/report/README.md)。报告编译沿用经过核对的实测 JSON，不会重新运行 FK/IK 仿真。

交付文件 / Deliverables:

- `assignments/assignment_1_kinematics/forward_kinematics.py`
- `assignments/assignment_1_kinematics/inverse_kinematics.py`
- `output/pdf/assignment_1_report.pdf`：三张 D-H 表、关节 signs、方法与实测比较结果 / D-H tables, signs, method and measured results.
- `output/assignment_1_latex_source.zip`：可在本地或 Overleaf 编译的完整 LaTeX 源码 / Portable LaTeX source, bibliography, generated tables and vector figure.
- `output/assignment_1_submission.zip`：按题目要求整理的提交包 / Submission package with both completed scripts, the report, and measured results.
- `assignments/assignment_1_kinematics/results/fk_results.json`
- `assignments/assignment_1_kinematics/results/ik_results.json`

JSON 文件记录实测误差和通过状态，报告生成脚本使用这些数据。代码和报告准备完成后，请自行核对并通过课程要求的渠道提交。

The JSON files contain measured errors and pass/fail status and supply the report data. Review the completed code and report, then submit them through the course's required channel; this project does not submit to the university automatically.
