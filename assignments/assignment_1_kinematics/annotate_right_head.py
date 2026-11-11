"""Prepare native-PDF annotation sheets for the right leg and head drawings.

Handout coordinates are manually registered in PDF points. They identify
schematic joint-axis intersections, not a metric projection of the XML model.
Exact axes and dimension values are computed from the verified MDH frames.
"""
from pathlib import Path
import json
import shutil
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from assignments.assignment_1_kinematics.draw_dh_diagrams import frames, validate_geometry
from assignments.assignment_1_kinematics.forward_kinematics import parameters

REPORT = Path(__file__).resolve().parent / "report"
SOURCES = REPORT / "annotations"
ASSETS = REPORT / "assets"
WORK = ROOT / ".tmp" / "on-source-annotations"

PREAMBLE = r"""\documentclass[11pt]{article}
\usepackage[T1]{fontenc}
\renewcommand{\rmdefault}{ptm}
\usepackage[paperwidth=297mm,paperheight=210mm,margin=0mm]{geometry}
\usepackage{amsmath,amssymb,booktabs,graphicx,tikz}
\usetikzlibrary{arrows.meta,calc}
\definecolor{ink}{HTML}{243746}
\definecolor{axa}{HTML}{D17B14}
\definecolor{axd}{HTML}{963D98}
\definecolor{axx}{HTML}{1765B0}
\definecolor{axy}{HTML}{C43C39}
\definecolor{axz}{HTML}{22854E}
\pagestyle{empty}
\begin{document}\null
\begin{tikzpicture}[remember picture,overlay,x=1mm,y=-1mm,
  originmark/.style={circle,draw=ink,fill=white,inner sep=1.3pt,line width=.65pt},
  dimlabel/.style={fill=white,inner sep=1.2pt,font=\small},
  al/.style={-{Latex[length=1.7mm]},draw=axa,line width=1.0pt},
  dl/.style={-{Latex[length=1.7mm]},draw=axd,line width=1.0pt}]
\begin{scope}[shift={(current page.north west)}]
"""
ENDING = "\\end{scope}\n\\end{tikzpicture}\n\\end{document}\n"


def coord(point):
    return f"({point[0]:.3f},{point[1]:.3f})"


def node(point, content, options="anchor=north west,text=ink"):
    return rf"\node[{options}] at {coord(point)} {{{content}}};" + "\n"


def direction(vector):
    index = int(np.argmax(np.abs(vector)))
    return ("+" if vector[index] >= 0 else "-") + "XYZ"[index]


def prepare(part):
    poses = frames(part)
    dh, signs, _ = parameters(part)
    is_head = part == "head"
    title = "Head and neck" if is_head else "Right leg"
    scale = .43 if is_head else .44
    offset = np.array([4., 41.]) if is_head else np.array([24., 40.])
    if is_head:
        raw = {"B": (175., 261.5), "O0": (175., 203.5), "O1": (175., 203.5),
               "P2": (175., 149.), "O2": (120., 149.), "O3": (120., 99.),
               "O4": (77., 124.), "O5": (77., 35.)}
        raw.update(P1=raw["O0"], P3=raw["O2"], P4=raw["O3"], P5=raw["O4"])
        label_offsets = {"B": (16, 11), "O1": (20, 14), "P2": (17, -10),
                         "O2": (-1, 22), "O3": (38, -7), "O4": (-9, 23), "O5": (60, -9)}
        paths = [("O1", "P2", "a_1", "al", (7, 0)),
                 ("P2", "O2", "d_2", "dl", (0, 6)),
                 ("O2", "O3", "d_3", "dl", (7, 1)),
                 ("O3", "O4", "d_4", "dl", (-25, 57)),
                 ("O4", "O5", "d_5", "dl", (-7, 0))]
        axis_lines = [("O1", (-22, 0), (22, 0)), ("O2", (-20, 0), (35, 0)),
                      ("O3", (0, -18), (0, 24)), ("O4", (-16, 9.3), (28, -16.3))]
        zero_note = (r"$a_0=a_2=a_3=a_4=0$; $d_1=0$.\\"
                     r"$P_1=O_0=O_1$, $P_3=O_2$, $P_4=O_3$, $P_5=O_4$.")
        origin_note = (r"$O_0=O_1$ at the neck axis; their orientations differ. "
                       r"$P_2$ is the pitch-axis point vertically above $O_1$. "
                       r"$O_2$ is shifted along the head-pitch axis. "
                       r"$O_3$ and $O_4$ lie on extended yaw and roll axes.")
        caveat = (r"The base-to-neck transform $^BT_0$ is separate. "
                  r"The handout's $d_1=92.021$ mm is not the D-H $d_1=0$.")
    else:
        raw = {"O0": (156., 45.5), "P1": (94., 30.), "O1": (94., 70.5),
               "O2": (137., 103.), "O3": (69.5, 103.), "O4": (69.5, 158.5),
               "P5": (69.5, 214.5), "O5": (105.5, 214.5),
               "P6": (125.5, 229.5), "O6": (125.5, 266.5)}
        raw.update(P2=raw["O1"], P3=raw["O2"], P4=raw["O4"])
        label_offsets = {"O0": (16, -23), "P1": (-16, -10), "O1": (-30, 0),
                         "O2": (27, -4), "O3": (-26, 0), "O4": (-26, 0),
                         "P5": (-25, -9), "O5": (35, -8), "P6": (16, -3), "O6": (25, 11)}
        paths = [("O0", "P1", "a_0", "al", (0, -6)),
                 ("P1", "O1", "d_1", "dl", (6, 0)),
                 ("O1", "O2", "d_2", "dl", (4, -5)),
                 ("O2", "O3", "d_3", "dl", (37, 10)),
                 ("O3", "O4", "a_3", "al", (-7, 0)),
                 ("O4", "P5", "a_4", "al", (-7, 0)),
                 ("P5", "O5", "d_5", "dl", (0, -6)),
                 ("O5", "P6", "a_5", "al", (5, -4)),
                 ("P6", "O6", "d_6", "dl", (17, 0))]
        axis_lines = [("O1", (0, -24), (0, 15)), ("O2", (-29, -21.9), (14, 10.6)),
                      ("O3", (-14, 0), (39, 0)), ("O4", (-14, 0), (20, 0)),
                      ("O5", (-26, 0), (14, 0))]
        zero_note = (r"$a_1=a_2=0$; $d_4=0$.\\"
                     r"$P_2=O_1$, $P_3=O_2$, $P_4=O_4$.")
        origin_note = (r"$O_1$: yaw/roll axis intersection. "
                       r"$O_2$: roll/hip-pitch axis intersection. "
                       r"$P_5$ and $O_5$ share the ankle axis. "
                       r"$P_6$ is the auxiliary corner for the D-H translation order.")
        caveat = (r"$O_0=B$. The first vertical segment ends at $O_1$; "
                  r"$d_1=-0.0741$ mm. Cylinder centres are not assumed to be D-H origins.")
    points = {name: offset + scale * np.asarray(point) for name, point in raw.items()}
    text = PREAMBLE
    text += node((10, 8), rf"{title}: origins, D-H distances and axes", "anchor=north west,text=ink,font=\\bfseries\\Large")
    text += node((10, 20), r"Direct annotations on the original assignment drawing. Zero pose; distances in mm.", "anchor=north west,text=ink,font=\\small")
    text += r"\draw[ink!30] (10,29)--(287,29);" + "\n"
    width = scale * (278 if is_head else 216)
    text += node(offset, rf"\includegraphics[width={width:.3f}mm]{{../assets/assignment_{part}_original.pdf}}", "anchor=north west,inner sep=0pt,opacity=.70")
    for name, before, after in axis_lines:
        p = points[name]
        text += rf"\draw[ink!50,densely dashed,line width=.5pt] {coord(p+before)}--{coord(p+after)};" + "\n"
    for start, end, label, style, shift in paths:
        p, q = points[start], points[end]
        text += rf"\draw[{style}] {coord(p)}--{coord(q)};" + "\n"
        if np.linalg.norm(shift) > 20:
            if is_head and label == "d_4":
                m, e = (p+q)/2, (p+q)/2+shift
                text += rf"\draw[ink!65,line width=.4pt] {coord(m)}--(7,{m[1]:.3f})--(7,{e[1]:.3f})--{coord(e)};" + "\n"
            else:
                text += rf"\draw[ink!65,line width=.4pt] {coord((p+q)/2)}--{coord((p+q)/2+shift)};" + "\n"
        text += node((p + q) / 2 + shift, rf"${label}^{{\rm DH}}$", f"dimlabel,text={'axa' if style=='al' else 'axd'}")
    for name, shift in label_offsets.items():
        p = points[name]
        text += rf"\node[originmark] at {coord(p)} {{}};" + "\n"
        if name == "O1" and is_head:
            label = r"$O_0=O_1$"
        elif name == "O0":
            label = r"$O_0=O_B$"
        elif name.startswith("O"):
            label = rf"$O_{{{name[1:]}}}$" + (" (site)" if int(name[1:]) == len(dh) else "")
        elif name.startswith("P"):
            label = rf"$P_{{{name[1:]}}}$"
        else:
            label = "$B$"
        q = p + shift
        if is_head and name == "O4":
            text += rf"\draw[ink!65,line width=.45pt] {coord(p)}--(17,{p[1]:.3f})--(17,{q[1]:.3f})--{coord(q)};" + "\n"
        else:
            text += rf"\draw[ink!65,line width=.45pt] {coord(p)}--{coord(q)};" + "\n"
        if name.startswith("O"):
            idx = int(name[1:])
            axislabels = [direction(poses[idx][:3, j]) for j in range(3)]
            if idx == 0 and not is_head:
                axislabels = [r"-sX-cY", r"cX-sY", "+Z"]
            coloured = [rf"\textcolor{{{colour}}}{{${axis}_{{{idx}}}={value}$}}" for axis, value, colour in zip("xyz", axislabels, ("axx", "axy", "axz"))]
            label = r"\shortstack{" + label + r"\\{\fontsize{7}{8}\selectfont " + r"\;".join(coloured) + "}}"
            if is_head and name == "O1":
                label = (r"\shortstack{$O_0=O_1$\\{\fontsize{7}{8}\selectfont "
                         r"\textcolor{axx}{$x_1=+Z$}\;\textcolor{axy}{$y_1=-X$}\;\textcolor{axz}{$z_1=-Y$}}"
                         r"\\{\fontsize{7}{8}\selectfont \textcolor{axx}{$x_0=+X$}\;\textcolor{axy}{$y_0=+Y$}\;\textcolor{axz}{$z_0=+Z$}}}")
            text += node(q, label, "fill=white,draw=ink!25,rounded corners=.6mm,inner sep=2pt,text=ink,font=\\small")
        else:
            text += node(q, label, "fill=white,inner sep=1pt,text=ink,font=\\small")
    text += node((8, 182), r"\textcolor{axa}{Orange: $a_{i-1}^{\rm DH}$}\quad\textcolor{axd}{Purple: $d_i^{\rm DH}$}", "anchor=north west,text=ink,font=\\small")
    text += node((8, 190), r"Original black $d$ labels are handout dimensions. Coloured superscript DH labels are table parameters.", "anchor=north west,text=ink,text width=117mm,font=\\footnotesize")
    # An exact orientation/distance key is necessary because the handout's
    # perspective is schematic, not a single metric projection.
    text += node((137, 34), r"\textbf{Read each row as a two-step path}", "anchor=north west,text=ink,font=\\normalsize")
    text += node((137, 43), r"$O_{i-1}\xrightarrow[+x_{i-1}]{a_{i-1}}P_i\xrightarrow[+z_i]{d_i}O_i$", "anchor=north west,text=ink,font=\\normalsize")
    text += node((137, 54), r"A negative value means travel opposite to that positive axis.", "anchor=north west,text=ink,font=\\small")
    rows = [r"\begin{tabular}{c|rr|cc}", r"$i$ & $a_{i-1}$ & $d_i$ & axis for $a$ & axis for $d$\\\hline"]
    for i, (alpha, a, theta, d) in enumerate(dh, 1):
        av = f"{a*1000:.4f}".rstrip("0").rstrip(".")
        dv = f"{d*1000:.4f}".rstrip("0").rstrip(".")
        xa = (r"-sX-cY" if part == "right" and i == 1 else direction(poses[i-1][:3, 0]))
        za = direction(poses[i][:3, 2])
        rows.append(rf"${i}$ & ${av}$ & ${dv}$ & ${xa}$ & ${za}$\\")
    rows.append(r"\end{tabular}")
    text += node((138, 63), "\n".join(rows), "anchor=north west,text=ink,font=\\small,inner sep=0pt")
    text += node((137, 102), zero_note, "anchor=north west,text=ink,font=\\footnotesize,align=left")
    # Full XYZ table, paired with explicit miniature triads rather than
    # pretending the original picture's cylinder perspective is metric.
    text += node((137, 114), r"\textbf{Full frame axes, expressed in base $B$}", "anchor=north west,text=ink,font=\\normalsize")
    xyz = [r"\begin{tabular}{c|ccc}", r"frame & \textcolor{axx}{$+x_i$} & \textcolor{axy}{$+y_i$} & \textcolor{axz}{$+z_i$}\\\hline"]
    for i, pose in enumerate(poses):
        axes = [direction(pose[:3, j]) for j in range(3)]
        if i == 0 and not is_head:
            axes = [r"-sX-cY", r"cX-sY", "+Z"]
        xyz.append(" & ".join([f"${i}$", *(f"${v}$" for v in axes)]) + r"\\")
    xyz.append(r"\end{tabular}")
    text += node((138, 124), "\n".join(xyz), "anchor=north west,text=ink,font=\\small,inner sep=0pt")
    text += node((231, 61), r"\textbf{Positive-axis triads}\newline Translated from their $O_i$", "anchor=north west,text=ink,font=\\footnotesize,text width=56mm,align=center")
    groups = [([0, 5], poses[0]), ([1], poses[1]), ([2], poses[2]), ([3], poses[3]), ([4], poses[4])] if is_head else [([0], poses[0]), ([1], poses[1]), ([2], poses[2]), ([3, 4], poses[3]), ([5], poses[5]), ([6], poses[6])]
    projection = np.array([[-.6, 1, 0], [.45, 0, -1]])
    for k, (indices, pose) in enumerate(groups):
        origin = np.array([244. + 30 * (k % 2), 87. + 29 * (k // 2)])
        index = ",".join(map(str, indices))
        text += node(origin + [0, -12], rf"$i={index}$", "text=ink,font=\\small")
        for j, (axis, colour) in enumerate(zip("xyz", ("axx", "axy", "axz"))):
            v = projection @ pose[:3, j]
            v /= np.linalg.norm(v)
            endpoint = origin + 6 * v
            tip = origin + 8.6 * v
            if indices == [0] and j == 0:
                tip += [0, 1]
            text += rf"\draw[-{{Latex[length=1.4mm]}},{colour},line width=.8pt] {coord(origin)}--{coord(endpoint)};" + "\n"
            text += node(tip, rf"${axis}_i$", f"text={colour},font=\\footnotesize,inner sep=0pt")
        text += rf"\fill[ink] {coord(origin)} circle[radius=.45mm];" + "\n"
    note_y = 166 if not is_head else 161
    if not is_head:
        text += node((138, 160), r"$s=\sin\beta$, $c=\cos\beta$, $\beta=\operatorname{atan2}(19,35)$.", "anchor=north west,text=ink,font=\\footnotesize")
    text += node((137, note_y), origin_note + " " + caveat, "anchor=north west,text=ink,text width=149mm,font=\\footnotesize,align=left")
    text += node((137, 192), r"Original: Assignment 1, Fig. " + ("4" if is_head else "3") + r". Schematic, not to scale; perspective is illustrative. Exact signs and directions are given in the tables. Markers are selected D-H origins, not CAD joint centres.", "anchor=north west,text=ink,text width=149mm,font=\\footnotesize,align=left")
    text += ENDING
    source = SOURCES / f"{part}_overlay.tex"
    source.write_text(text, encoding="utf-8")
    return source


def run(command, cwd):
    result = subprocess.run(command, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, encoding="utf-8", errors="replace")
    if result.returncode:
        raise RuntimeError(result.stdout)
    return result.stdout


def main():
    SOURCES.mkdir(parents=True, exist_ok=True)
    WORK.mkdir(parents=True, exist_ok=True)
    validate_geometry()
    for part in ("right", "head"):
        source = prepare(part)
        log = run(["pdflatex", "-interaction=nonstopmode", "-halt-on-error", "-no-shell-escape",
                   f"-output-directory={WORK}", source.name], SOURCES)
        log += run(["pdflatex", "-interaction=nonstopmode", "-halt-on-error", "-no-shell-escape",
                    f"-output-directory={WORK}", source.name], SOURCES)
        (WORK / f"{part}-stdout.txt").write_text(log, encoding="utf-8")
        destination = ASSETS / f"assignment_{part}_annotated.pdf"
        shutil.copyfile(WORK / f"{part}_overlay.pdf", destination)
        run(["pdftoppm", "-r", "150", "-singlefile", "-png", str(destination),
             str(ASSETS / f"assignment_{part}_annotated")], ROOT)
        print(destination)


if __name__ == "__main__":
    main()
