"""Build the submission PDF from completed DH tables and measured JSON results.

Run from the project root after FK and the full headless IK comparison.
Requires requirements-report.txt. No simulation results are generated or altered here.
"""
import json
from pathlib import Path
import platform
import sys
from zipfile import ZIP_DEFLATED, ZipFile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import scipy
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    Image, PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle,
)

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from assignments.assignment_1_kinematics.forward_kinematics import parameters

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
OUTPUT = ROOT / "output" / "pdf"
NAVY = colors.HexColor("#183C55")
TEAL = colors.HexColor("#127D84")
GRAY = colors.HexColor("#52616D")
LIGHT = colors.HexColor("#EDF3F5")


def load_results():
    fk = json.loads((RESULTS / "fk_results.json").read_text())
    ik = json.loads((RESULTS / "ik_results.json").read_text())
    if not (fk["passed"] and ik["passed"] and ik["completed"]):
        raise ValueError("Both FK and a complete IK comparison must pass first")
    if (ik["cycles_completed"], ik["samples_per_cycle"]) != (5, 101):
        raise ValueError("The report requires the supplied full 5 x 101 comparison")
    if len(ik["measurements"]) != 5 * 101 * 3:
        raise ValueError("Expected 1,515 measured chain poses")
    for chain in ("left", "right", "head"):
        if fk["chains"][chain]["samples"] != 100:
            raise ValueError("FK requires 100 configurations per chain")
    return fk, ik


def register_fonts():
    font_dir = Path(matplotlib.get_data_path()) / "fonts" / "ttf"
    for name, filename in (("Report", "DejaVuSans.ttf"),
                           ("ReportBold", "DejaVuSans-Bold.ttf"),
                           ("ReportMono", "DejaVuSansMono.ttf")):
        pdfmetrics.registerFont(TTFont(name, str(font_dir / filename)))
    pdfmetrics.registerFontFamily("Report", normal="Report", bold="ReportBold")


def styles():
    return {
        "title": ParagraphStyle("title", fontName="ReportBold", fontSize=21,
                                leading=26, textColor=NAVY, spaceAfter=8),
        "subtitle": ParagraphStyle("subtitle", fontName="Report", fontSize=9,
                                   leading=13, textColor=GRAY, spaceAfter=14),
        "heading": ParagraphStyle("heading", fontName="ReportBold", fontSize=12,
                                  leading=16, textColor=TEAL, spaceBefore=12,
                                  spaceAfter=6),
        "body": ParagraphStyle("body", fontName="Report", fontSize=9,
                               leading=13, spaceAfter=7, textColor=NAVY),
        "small": ParagraphStyle("small", fontName="Report", fontSize=8,
                                leading=11, spaceAfter=6, textColor=GRAY),
        "code": ParagraphStyle("code", fontName="ReportMono", fontSize=8,
                               leading=12, spaceAfter=7, textColor=NAVY),
        "cell": ParagraphStyle("cell", fontName="Report", fontSize=8.6,
                               leading=11, textColor=NAVY, alignment=TA_CENTER),
    }


def number(value):
    if np.isclose(value, 0, atol=1e-15):
        return "0"
    if np.isclose(value, np.hypot(.019, .035), atol=1e-15):
        return "r"
    return f"{value:.7f}".rstrip("0").rstrip(".")


def angle(value):
    beta = np.arctan2(19, 35)
    for actual, label in ((0, "0"), (np.pi / 2, "pi/2"),
                          (-np.pi / 2, "-pi/2"), (np.pi, "pi"),
                          (-beta, "-beta"), (np.pi + beta, "pi + beta")):
        if np.isclose(value, actual, atol=1e-14):
            return label
    return f"{value:.10f}"


def table(rows, widths, sty):
    cells = [[Paragraph(str(cell), sty["cell"]) for cell in row] for row in rows]
    result = Table(cells, colWidths=widths, repeatRows=1, hAlign="LEFT")
    result.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), LIGHT),
        ("LINEBELOW", (0, 0), (-1, 0), .8, TEAL),
        ("LINEBELOW", (0, -1), (-1, -1), .6, TEAL),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#F8FAFB")]),
        ("TOPPADDING", (0, 0), (-1, -1), 5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
    ]))
    return result


def dh_table(chain, sty):
    dh, signs, _ = parameters(chain)
    rows = [["i", "alpha_(i-1) [rad]", "a_(i-1) [m]", "d_i [m]", "theta_i [rad]"]]
    for i, (alpha, a, offset, d) in enumerate(dh):
        theta = angle(offset)
        if i < len(signs):
            term = ("-" if signs[i] < 0 else "") + f"q{i + 1}"
            theta = term if theta == "0" else (
                term + " - " + theta[1:] if theta.startswith("-") else term + " + " + theta)
        rows.append([str(i + 1) if i < len(signs) else f"{i + 1} fixed",
                     angle(alpha), number(a), number(d), theta])
    return table(rows, [48, 112, 102, 95, 142], sty)


def trajectory_plot(ik):
    plt.rcParams.update({"font.size": 9, "axes.spines.top": False,
                         "axes.spines.right": False, "figure.dpi": 160})
    fig, axes = plt.subplots(2, 1, figsize=(7.8, 4.4), sharex=True, constrained_layout=True)
    for part, color in zip(("left", "right", "head"), ("#127D84", "#DF7E32", "#555AA7")):
        samples = [m for m in ik["measurements"] if m["chain"] == part]
        x = np.arange(1, len(samples) + 1)
        for ax, key in zip(axes, ("position_error_m", "orientation_error_rad")):
            values = np.array([m[key] for m in samples])
            ax.semilogy(x, np.maximum(values, 1e-16), label=part, color=color, linewidth=1)
    for ax, tolerance in zip(axes, (1e-5, 1e-4)):
        ax.axhline(tolerance, color="#A83C3C", linestyle="--", linewidth=1,
                   label="required limit")
        for boundary in (101, 202, 303, 404):
            ax.axvline(boundary + .5, color="#DDE4E7", linewidth=.7)
        ax.grid(axis="y", alpha=.18)
        ax.set_xlim(1, 505)
    axes[0].set_ylabel("Position error [m]")
    axes[1].set_ylabel("Orientation error [rad]")
    axes[1].set_xlabel("Target index per chain (5 cycles x 101 phases)")
    axes[0].legend(loc="lower right", ncol=4, fontsize=7, frameon=False)
    path = RESULTS / "ik_error_plot.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def footer(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(colors.HexColor("#DCE5E9"))
    canvas.line(48, 40, A4[0] - 48, 40)
    canvas.setFont("Report", 7.5)
    canvas.setFillColor(GRAY)
    canvas.drawString(48, 27, "ROSE5710  |  Assignment 1: Kinematics")
    canvas.drawRightString(A4[0] - 48, 27, str(doc.page))
    canvas.restoreState()


def main():
    fk, ik = load_results()
    register_fonts()
    sty = styles()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    story = []

    def p(text, style="body"):
        story.append(Paragraph(text, sty[style]))

    p("Assignment 1: Kinematics", "title")
    p("ROSE5710: Fundamental Robotics and Systems  |  6 November 2026", "subtitle")
    p("Modified D-H convention and leg chains", "heading")
    p("The transforms describe each site relative to the robot base. All translations below "
      "are in metres and all joint angles are in radians. The final row in each table is fixed.")
    p("A_i = Rx(alpha_(i-1)) Tx(a_(i-1)) Rz(theta_i) Tz(d_i)<br/>"
      "theta_i = s_i q_i + theta0_i;  T_B,site = T_B,0 A_1 ... A_n", "code")
    p("Define r = sqrt(0.019² + 0.035²) = 0.03982461550348 m and "
      "beta = atan2(19, 35) = 0.49734271634682 rad. Code stores each row as "
      "[alpha, a, theta0, d]; these tables follow the question's [alpha, a, d, theta] order.", "small")
    p("Left leg (5 joints + foot site)", "heading")
    p("T_B,0 = Rz(pi/2 + beta); signs s = [1, -1, -1, -1, -1].", "code")
    story.append(dh_table("left", sty))
    p("Right leg (5 joints + foot site)", "heading")
    p("T_B,0 = Rz(-pi/2 - beta); signs s = [1, -1, 1, -1, -1].", "code")
    story.append(dh_table("right", sty))
    story.append(Spacer(1, 10))
    p("Frame assignment: in the zero pose, the leg D-H joint axes z1...z5 are "
      "[+Z, +X, +Y, +Y, +Y]. Each x axis follows the common normal; virtual origins "
      "are selected at the yaw/roll and roll/pitch axis intersections. The final fixed row "
      "aligns the foot-site axes with the base axes at zero joint angles.", "small")
    p("Geometry: the roll-axis height is 45.9409 - 46.015 = -0.0741 mm. Shifting "
      "origins along the parallel pitch axes combines the small CAD offsets into the "
      "specified final lateral distances: left 19.75 mm and right 17.35 mm. "
      "The two legs therefore require different final lateral offsets.", "small")

    story.append(PageBreak())
    p("Head chain and inverse kinematics", "title")
    p("Head and neck (4 joints + head site)", "heading")
    p("T_B,0 has identity rotation and p = [0.001, 0.01915, 0.0900009] m; "
      "signs s = [1, 1, 1, 1].", "code")
    story.append(dh_table("head", sty))
    story.append(Spacer(1, 10))
    p("In the zero pose the head D-H joint axes are [-Y, -Y, +Z, -X], matching the "
      "MuJoCo positive directions. The neck-to-head-pitch separation is 66 mm. "
      "The 92.021 mm base-to-neck distance is already included in T_B,0 and must "
      "not be added again. The effective vertical separation from the head-pitch axis "
      "to the roll axis is 37.85 mm; the net roll-axis offset is 5 mm.")
    p("Zero-pose cross-check", "heading")
    story.append(table([
        ["Site", "Base-frame position [m]", "Rotation"],
        ["left foot", "[-0.03456, 0.08940, -0.1935991]", "I"],
        ["right foot", "[-0.03456, -0.09180, -0.1935991]", "I"],
        ["head", "[0.00600, 0.00010, 0.2363009]", "I"],
    ], [92, 315, 92], sty))
    p("Numerical IK method", "heading")
    p("For a target pose (p_t, R_t) and its FK prediction (p(q), R(q)), the position "
      "residual is target minus prediction. The orientation residual is the axis-angle "
      "rotation vector, rather than an Euler-angle difference.")
    p("e_p = p_t - p(q);  e_R = rotvec(R_t R(q)^T)<br/>"
      "E(q) = ||10 e_p||² + ||e_R||²<br/>"
      "g_i = [E(q + h e_i) - E(q - h e_i)] / (2h)<br/>"
      "q_next = clip(q - alpha g, q_min, q_max)", "code")
    p("Use h = 1e-6 rad, fixed alpha = 0.1, at most 5,000 iterations per target, "
      "and stop at E &lt; 1e-8. Joint limits are read from MuJoCo. Each solve starts "
      "from the current joint angles, and only the selected chain is updated. "
      "The stopping rule bounds position error below 1e-5 m and orientation error below 1e-4 rad.")
    p("Supplied trajectories", "heading")
    p("Both feet move forward and upward by up to 20 mm at fixed orientation; "
      "the left uses sin²(pi phase) and the right uses cos²(pi phase). The head follows "
      "a neck-centred arc of 0.1 sin²(pi phase) rad, updating both position and orientation. "
      "The base stays fixed and every target is checked against MuJoCo site poses.")

    story.append(PageBreak())
    p("Measured verification results", "title")
    p(f"Python {platform.python_version()}  |  NumPy {np.__version__}  |  "
      f"SciPy {scipy.__version__}  |  MuJoCo {mujoco.__version__}", "subtitle")
    p("FK: 100 joint-limit-uniform configurations per chain with seed 0 (300 total). "
      "IK: all three supplied trajectories, 5 complete cycles and 101 phases per cycle "
      "(1,515 measured chain poses). Headless execution changes only rendering and delay.")
    rows = [["Check", "Chain", "Max position [m]", "Max rotation [rad]", "Result"]]
    for label, result in (("FK", fk), ("IK", ik)):
        for part in ("left", "right", "head"):
            item = result["chains"][part]
            rows.append([label, part, f"{item['max_position_error_m']:.6e}",
                         f"{item['max_orientation_error_rad']:.6e}", "PASS"])
    story.append(table(rows, [48, 68, 150, 155, 78], sty))
    story.append(Spacer(1, 8))
    p("Required maxima: FK &lt; 1e-6 m and &lt; 1e-5 rad; "
      "IK &lt; 1e-5 m and &lt; 1e-4 rad. Every measured value passes. "
      "Six independent regression tests also cover base-frame invariance, pose-error "
      "weighting, reachable IK, joint limits, and preservation of other chains.", "small")
    plot = trajectory_plot(ik)
    story.append(Image(str(plot), width=499, height=281.5))
    p("All 505 target indices per chain are shown. Dashed lines mark the required maxima; "
      "vertical lines separate cycles. Values below 1e-16 are floored only for the log plot. "
      "JSON files retain the original measured values.", "small")
    p("Reproduce from the project root", "heading")
    p("python assignments/assignment_1_kinematics/forward_kinematics.py<br/>"
      "python assignments/assignment_1_kinematics/inverse_kinematics.py --headless<br/>"
      "python -m unittest assignments.assignment_1_kinematics.test_kinematics", "code")
    p("Use the Python 3.11 environment and requirements.txt listed in README.md. "
      "The results directory contains the per-configuration FK data and all IK phase errors. "
      "Add --output-json to either script to regenerate its results; build_report.py "
      "rebuilds this PDF from those files.", "small")

    path = OUTPUT / "assignment_1_report.pdf"
    SimpleDocTemplate(str(path), pagesize=A4, leftMargin=48, rightMargin=48,
                      topMargin=43, bottomMargin=52,
                      title="ROSE5710 Assignment 1: Kinematics",
                      author="", subject="Modified D-H derivation and FK/IK verification").build(
                          story, onFirstPage=footer, onLaterPages=footer)
    print(path)
    package = OUTPUT.parent / "assignment_1_submission.zip"
    with ZipFile(package, "w", compression=ZIP_DEFLATED) as archive:
        for source in (HERE / "forward_kinematics.py", HERE / "inverse_kinematics.py",
                       path, RESULTS / "fk_results.json", RESULTS / "ik_results.json",
                       RESULTS / "environment.json"):
            archive.write(source, source.name)
        archive.writestr("README.txt", (
            "ROSE5710 Assignment 1: Kinematics\n\n"
            "Submission: both completed Python files, the PDF containing the three\n"
            "modified D-H tables and signs, and measured FK/IK comparison results.\n\n"
            "The two scripts run within the original Open_Duck_Playground project.\n"
            "Place them in assignments/assignment_1_kinematics/. The full runnable\n"
            "project, dependencies, tests and report builder are available at:\n"
            "https://github.com/David0412-sun/5710_Assignment1\n\n"
            "Use Python 3.11 and follow the project README.\n"
        ))
    print(package)


if __name__ == "__main__":
    main()
