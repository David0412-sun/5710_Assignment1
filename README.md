# ROSE5710 Assignment 1: Kinematics

本项目完成 Open Duck Mini 双足机器人的左腿、右腿与头颈链运动学作业：Modified D-H 推导、正运动学（FK）、有限差分梯度下降逆运动学（IK），以及与 MuJoCo site 位姿的比较。

This assignment implements modified D-H forward kinematics and bounded, fixed-step gradient-descent inverse kinematics for both legs and the head. Verification compares both position and orientation with MuJoCo.

## 环境 / Setup

本机 Windows 使用项目独立的 Python 3.11 环境 `.venv\Scripts\python.exe`。作业所需的轻量 MuJoCo 环境不依赖 JAX、CUDA 或训练环境，`pyproject.toml` 的默认依赖也已改为作业依赖。

在 PowerShell 中进入本项目目录。首次配置或重建环境：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\setup_windows.ps1 -Report
```

配置脚本复用 Python 3.11 的 Windows `.venv`；环境不存在时优先使用 `py -3.11`，否则通过 `uv` 下载并创建。已有不兼容的 `.venv` 会明确报错，需要先重命名再重新配置。通过 `uv` 下载的解释器及其缓存保存在项目 `.tmp` 内；依赖安装到 `.venv`，pip 使用默认下载缓存。无需激活虚拟环境或更改系统执行策略。`-Report` 同时安装报告绘图依赖。本机通过 `uv` 配置，不要手动移动或删除 `.tmp/python`，因为虚拟环境依赖其中的解释器。

常用入口（这些命令只对当前 PowerShell 子进程使用 `Bypass`）：

```powershell
# 12 项测试：6 项运动学 + 6 项报告构建
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_assignment.ps1 test
# FK + 完整 IK + 测试；成功后更新 results 和本机 environment.json
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_assignment.ps1 verify
# 查看模型或播放 IK 轨迹（关闭窗口退出）
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_assignment.ps1 view
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_assignment.ps1 ik
# 使用已记录的测量结果生成报告，需要 TeX 引擎
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_assignment.ps1 report
```

若使用 VS Code / PyCharm，将解释器选为本项目 `.venv\Scripts\python.exe`。完整验证需进行大量有限差分计算；控制台每完成一个周期显示进度。

Linux/macOS 新环境仍可按以下方式配置（下文 Windows 命令中的 `.venv\Scripts\python.exe` 相应替换为 `.venv/bin/python`）：

```bash
python3.11 -m venv .venv
.venv/bin/python -m pip install -r assignments/assignment_1_kinematics/requirements.txt
```

上游强化学习训练依赖保存在 `pyproject.toml` 的可选 `training` 组，作业不使用该组，其 CUDA 环境不属于本次 Windows 配置。

## 课程核对

核对依据为本机 ROSE5710 的 LS3、LS4、LS5 课件，以及 Git 历史 `7d70126` 的原始 TODO 模板。逐项结论和准确页码见 [COURSE_REVIEW.md](COURSE_REVIEW.md)。报告已区分课程术语中的“解析建模”和“数值 FK”，课程核对记录保留在上述独立文档中。

## D-H 约定 / Convention

每行变换为 `Rx(alpha) @ Tx(a) @ Rz(theta) @ Tz(d)`，其中 `theta = signs[i] * q[i] + theta_offset`；最后一行为不受关节角影响的固定 site 行。

The code stores rows as **`[alpha, a, theta_offset, d]`**, whereas the assignment's printed tables use **`[alpha, a, d, theta]`**. Translations are in metres and angles are in radians. The three tables and sign vectors are defined in `forward_kinematics.py` and listed in the report.

## 验证 / Run and compare

正运动学：每条链使用固定随机种子测试 100 个关节配置。位置误差要求小于 `1e-6 m`，姿态误差小于 `1e-5 rad`。

FK checks 100 seeded random configurations per chain:

```powershell
.\.venv\Scripts\python.exe assignments/assignment_1_kinematics/forward_kinematics.py
```

逆运动学：默认完整运行 5 个周期，每周期 101 个相位，跟踪左右脚和头部的原始目标轨迹。保留作业指定的中心差分 `h=1e-6`、固定步长 `alpha=0.1`、最多 5000 次迭代及 `cost < 1e-8` 停止条件。位置误差要求小于 `1e-5 m`，姿态误差小于 `1e-4 rad`。

IK runs the supplied trajectories for five cycles with 101 phases each. The graphical version opens the MuJoCo viewer; headless mode removes rendering and its delay while preserving the algorithm, targets and tolerances:

```powershell
# Graphical viewer / 图形界面
.\.venv\Scripts\python.exe assignments/assignment_1_kinematics/inverse_kinematics.py

# Full numerical comparison / 完整数值验证
.\.venv\Scripts\python.exe assignments/assignment_1_kinematics/inverse_kinematics.py --headless
```

macOS 图形查看器请先激活环境，然后用 `mjpython` 运行：

```bash
source .venv/bin/activate
mjpython assignments/assignment_1_kinematics/inverse_kinematics.py
```

`--cycles`、`--samples` 与 `--parts` 可用于局部调试；正式比较保留默认参数。完整运行并达到全部误差要求时，脚本返回退出码 0。

Use `--cycles`, `--samples` and `--parts` for smaller diagnostic runs. Keep the defaults for the full comparison. Successful complete comparisons exit with status 0.

## 测试与报告 / Tests and report

报告直接给出三条链的 D-H 坐标图、参数及轴向表、基座变换和 signs，以及 FK/IK 结果表和实测误差曲线。原有机器人渲染素材保留为独立参考，不属于报告内容；图像生成和复现方式见 [报告说明](assignments/assignment_1_kinematics/report/README.md)。

报告采用精简答案式：A4 单栏，保留原有 Times（`ptm`）正文字体和 Courier（`pcr`）代码字体，以必要答案、表格和图像为主，说明保持简短，不设参考文献部分。正文源码位于 `assignments/assignment_1_kinematics/report/assignment_1_report.tex`，可直接编辑。

The report presents the required answers, tables and figures directly in a
single-column A4 layout, retaining the Times (`ptm`) and Courier (`pcr`) fonts.
It keeps explanations brief and has no bibliography. The LaTeX build checks
for errors, unresolved cross-references, missing glyphs, and overfull boxes.

报告小组 / **Group11**:

| 姓名 / Name | 学号 / Student ID |
| --- | --- |
| Sun Hejia | 1155262875 |
| Long Yifan | 1155262899 |
| Liang Zhuolun | 1155262896 |
| Wang Kaiyuan | 1155269457 |

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s assignments/assignment_1_kinematics -p 'test_*.py' -v

# Optional report-generation dependencies / 可选报告依赖
powershell -NoProfile -ExecutionPolicy Bypass -File .\setup_windows.ps1 -Report
.\.venv\Scripts\python.exe assignments/assignment_1_kinematics/build_report.py
```

编译需要 Tectonic 或 TeX Live/MiKTeX。构建脚本会检测当前工作区可用的编译器；首次运行可能需要下载报告使用的常规宏包和字体。也可使用 `--engine tectonic`、`--engine latexmk` 或 `--engine pdflatex` 指定引擎。完整源码编译说明见 [LaTeX 源码说明](assignments/assignment_1_kinematics/report/README.md)。报告编译沿用经过核对的实测 JSON，不会重新运行 FK/IK 仿真。

交付文件 / Deliverables:

- `assignments/assignment_1_kinematics/forward_kinematics.py`
- `assignments/assignment_1_kinematics/inverse_kinematics.py`
- `output/pdf/assignment_1_report_group_11.pdf`：三张 D-H 表、关节 signs、方法与实测比较结果 / D-H tables, signs, method and measured results.
- `output/assignment_1_latex_source.zip`：可在本地或 Overleaf 编译的完整 LaTeX 源码 / Portable LaTeX source, generated tables and vector figure.
- `output/assignment_1_submission_group_11.zip`：按题目要求整理的提交包 / Submission package with both completed scripts, the report, and measured results.
- `assignments/assignment_1_kinematics/results/fk_results.json`
- `assignments/assignment_1_kinematics/results/ik_results.json`

JSON 文件记录实测误差和通过状态，报告生成脚本使用这些数据。代码和报告准备完成后，请自行核对并通过课程要求的渠道提交。

更新正式测量结果请运行 `run_assignment.ps1 verify`，它会同时记录真实运行环境、算法参数和源码指纹。单独运行 FK/IK 的 `--output-json` 适合导出到临时路径做诊断；不要只覆盖正式结果而保留旧的 `environment.json`。源码指纹采用 LF 规范化，避免 Windows CRLF 换行造成误报。

The JSON files contain measured errors and pass/fail status and supply the report data. Review the completed code and report, then submit them through the course's required channel; this project does not submit to the university automatically.
