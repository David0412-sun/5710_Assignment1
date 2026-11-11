"""Draw exact zero-pose MDH joint/link models and their full XYZ frames.

All chains use the same oblique base projection as the assignment illustrations:
+Z up, +Y right, +X down-left. Frame origins retain their computed geometry;
axis glyph lengths and text offsets are only display choices. A DH origin may
slide along a physical joint axis, so the connected lines are equivalent links,
not a tracing of the CAD outline or the centres of its drawn cylinders.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".tmp" / "matplotlib"))
sys.path.insert(0, str(ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from assignments.assignment_1_kinematics.forward_kinematics import (
    forward_kinematics_transforms, joint_names, parameters,
)
from playground.open_duck_mini_v2.constants import FLAT_TERRAIN_XML

# Match the handout: x blue, y red, z green.
AXIS_COLORS = ("#1765B0", "#C43C39", "#22854E")
INK = "#34434B"
LINK = "#63717B"
PROJECTION = np.array([[-.60, 1., 0.], [-.45, 0., 1.]])


def frames(part):
    table, signs, base = parameters(part)
    return [base, *forward_kinematics_transforms(np.zeros(len(signs)), table, signs, base)]


def validate_geometry():
    """Check axis lines/sites against MuJoCo and each frame's handedness."""
    model = mujoco.MjModel.from_xml_path(str(FLAT_TERRAIN_XML))
    data = mujoco.MjData(model)
    data.qpos[:] = model.qpos0
    mujoco.mj_forward(model, data)
    base = model.body("base").id
    base_R = data.xmat[base].reshape(3, 3)
    base_p = data.xpos[base]
    report = {}
    for part in ("left", "right", "head"):
        poses = frames(part)
        signs = parameters(part)[1]
        axis_errors, line_errors = [], []
        for index, name in enumerate(joint_names(part), 1):
            joint = model.joint(name).id
            anchor = base_R.T @ (data.xanchor[joint] - base_p)
            axis = base_R.T @ data.xaxis[joint]
            pose = poses[index]
            axis_errors.append(float(np.linalg.norm(signs[index - 1] * pose[:3, 2] - axis)))
            line_errors.append(float(np.linalg.norm(np.cross(pose[:3, 3] - anchor, axis))))
        site = model.site("head" if part == "head" else part + "_foot").id
        site_pos = base_R.T @ (data.site_xpos[site] - base_p)
        site_rot = base_R.T @ data.site_xmat[site].reshape(3, 3)
        rotations = [pose[:3, :3] for pose in poses]
        result = {
            "max_joint_axis_error": max(axis_errors),
            "max_joint_line_distance_m": max(line_errors),
            "site_position_error_m": float(np.linalg.norm(poses[-1][:3, 3] - site_pos)),
            "site_rotation_matrix_error": float(np.linalg.norm(poses[-1][:3, :3] - site_rot)),
            "max_frame_orthonormal_error": max(float(np.linalg.norm(R.T @ R - np.eye(3))) for R in rotations),
            "max_frame_determinant_error": max(float(abs(np.linalg.det(R) - 1.)) for R in rotations),
            "max_frame_right_hand_error": max(float(np.linalg.norm(np.cross(R[:, 0], R[:, 1]) - R[:, 2])) for R in rotations),
        }
        if any(value >= 1e-10 for value in result.values()):
            raise ValueError(f"Invalid DH diagram geometry: {part}: {result}")
        # Column vectors expressed in B are printed for the report's axis table.
        result["frame_axes_in_base"] = [
            {"frame": index, "origin_mm": np.round(pose[:3, 3] * 1000, 6).tolist(),
             **{axis: np.round(pose[:3, column], 9).tolist()
                for column, axis in enumerate(("x", "y", "z"))}}
            for index, pose in enumerate(poses)
        ]
        report[part] = result
    return report


def arrow(ax, start, end, color, width=1.0, scale=7.0):
    ax.annotate("", xy=end, xytext=start,
                arrowprops={"arrowstyle": "-|>", "color": color, "lw": width,
                            "mutation_scale": scale, "shrinkA": 0, "shrinkB": 0},
                zorder=4)


def project(part=None):
    """Use one common oblique projection for both legs and the head."""
    return PROJECTION.copy()


def label(ax, text, point, offset, color=INK, size=8.2):
    ax.annotate(text, xy=point, xytext=np.asarray(point) + offset,
                fontsize=size, color=color, ha="center", va="center",
                bbox={"boxstyle": "round,pad=.08", "fc": "white", "ec": "none"},
                arrowprops={"arrowstyle": "-", "color": "#9AA3A8", "lw": .6,
                            "shrinkA": 2, "shrinkB": 2}, zorder=7)


def frame_groups(poses):
    groups = []
    for index, pose in enumerate(poses):
        for indices, rotation in groups:
            if np.allclose(rotation, pose[:3, :3], atol=1e-12):
                indices.append(index)
                break
        else:
            groups.append(([index], pose[:3, :3]))
    return groups


def draw_frame_key(fig, poses):
    """Orientation-only insets: no frame origin is moved in the chain model."""
    fig.text(.84, .893, "Frame axes", ha="center", fontsize=8.2, color=INK)
    groups = frame_groups(poses)
    row_height = .119
    for row, (indices, rotation) in enumerate(groups):
        ax = fig.add_axes([.705, .866 - (row + 1) * row_height, .28, row_height])
        ax.set_xlim(-.72, .72)
        ax.set_ylim(-.50, .94)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.text(.03, .89, "$i=" + ",".join(map(str, indices)) + "$",
                ha="center", va="top", fontsize=8.0, color=INK)
        for column, axis in enumerate(("x", "y", "z")):
            direction = PROJECTION @ rotation[:, column]
            direction /= np.linalg.norm(direction)
            # Frame 0 y/z have close projections: separate their arrowheads.
            length = (.31, .43, .26)[column] if indices == [0] else .32
            endpoint = length * direction
            arrow(ax, [0, 0], endpoint, AXIS_COLORS[column], width=.95, scale=6)
            textpoint = endpoint + .12 * direction
            if indices == [0] and column == 2:
                textpoint += [.28, -.04]
            if indices == [0] and column == 1:
                textpoint += [-.10, .02]
            ax.text(*textpoint, f"${axis}_i$", ha="center", va="center",
                    color=AXIS_COLORS[column], fontsize=8.0)
        ax.plot(0, 0, ".", ms=2, color=INK)
    fig.text(.84, .137, "Same directions\nwithin each group",
             ha="center", va="top", fontsize=7.0, color="#59666E", linespacing=1.2)
    fig.lines.append(plt.Line2D([.695, .695], [.15, .90], transform=fig.transFigure,
                               color="#D8DEE1", lw=.7))


def draw(part, destination):
    poses = frames(part)
    points = np.stack([pose[:3, 3] * 1000 for pose in poses]) @ PROJECTION.T
    n = len(joint_names(part))
    base_point = np.zeros(2)
    # The rigid base anchor is ell_0 even when frame 0 does not coincide with B.
    chain = np.vstack([base_point, points[1:]])
    fig = plt.figure(figsize=(3.35, 4.45))
    ax = fig.add_axes([.025, .17, .655, .705])
    ax.set_aspect("equal")
    ax.axis("off")
    ax.plot(chain[:, 0], chain[:, 1], color=LINK, lw=2.15, zorder=1)
    ax.scatter(points[1:n + 1, 0], points[1:n + 1, 1], s=16, color=INK, zorder=6)
    ax.scatter(*base_point, s=30, marker="D", facecolors="white", edgecolors=INK,
               linewidths=1.05, zorder=6)
    ax.scatter(*points[-1], s=34, marker="s", facecolors="white", edgecolors=INK,
               linewidths=1.05, zorder=6)

    if part == "left":
        limits = ((-36, 187), (-211, 61))
        joint_offsets = {1: (-25, 38), 2: (15, 35), 3: (21, 22), 4: (-31, 0), 5: (-30, -4)}
        link_offsets = [(-5, -27), (3, -21), (0, -18), (-17, 0), (-17, 0), (-18, 0)]
        site_offset, base_offset = (23, -17), (0, 24)
    elif part == "right":
        limits = ((-145, 65), (-211, 61))
        joint_offsets = {1: (1, 37), 2: (33, 13), 3: (-27, 20), 4: (-28, 0), 5: (-28, -2)}
        link_offsets = [(32, -9), (-3, -24), (0, -17), (19, 0), (18, 0), (18, 0)]
        site_offset, base_offset = (-26, -17), (29, -32)
    else:
        limits = ((-95, 97), (-24, 265))
        joint_offsets = {1: (32, -10), 2: (36, -4), 3: (37, 14), 4: (-44, -10)}
        link_offsets = [(-24, -3), (24, 0), (20, -2), (-48, 15), (-22, 0)]
        site_offset, base_offset = (27, 9), (25, -9)

    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    for index in range(1, n + 1):
        # Positive DH z_i can be opposite to the motor's positive axis.
        direction = PROJECTION @ poses[index][:3, 2]
        direction /= np.linalg.norm(direction)
        endpoint = points[index] + 24 * direction
        arrow(ax, points[index], endpoint, AXIS_COLORS[2], width=1.05)
        textpoint = endpoint + 7 * direction
        if part != "head" and index == 1:
            textpoint += [-9, -2]
        if part == "head" and index == 4:
            textpoint += [1, 3]
        ax.text(*textpoint, f"$z_{index}$", color=AXIS_COLORS[2], fontsize=8,
                ha="center", va="center", zorder=8)
        label(ax, f"$O_{index}$", points[index], joint_offsets[index])
    for index in range(n + 1):
        midpoint = (chain[index] + chain[index + 1]) / 2
        label(ax, f"$\\ell_{index}$", midpoint, link_offsets[index], size=8.5)
    label(ax, "$B$" if part == "head" else "$O_B=O_0$", base_point, base_offset)
    label(ax, f"$O_{n + 1}=S$", points[-1], site_offset)

    if part == "head":
        ax.text(52, 62, "$O_0=O_1$ at $J_1$", fontsize=7.3, color=INK, ha="center")
    draw_frame_key(fig, poses)
    fig.text(.5, .975, "Equivalent joint / link model", ha="center", va="top",
             fontsize=9.4, color=INK)
    fig.text(.5, .934, "Zero pose  |  common base projection", ha="center", va="top",
             fontsize=7.7, color="#59666E")
    fig.text(.03, .104, "$O_i$: DH origin on $J_i$    $S$: fixed site",
             ha="left", va="top", fontsize=7.5, color=INK)
    fig.text(.03, .069, "$\\ell_0$: base anchor    $\\ell_i$: equivalent rigid link",
             ha="left", va="top", fontsize=7.5, color=INK)
    fig.text(.03, .034, "Axes: x blue / y red / z green; arrows show + directions.",
             ha="left", va="top", fontsize=7.0, color="#59666E")
    for suffix in ("pdf", "png"):
        kwargs = {"dpi": 300} if suffix == "png" else {}
        fig.savefig(destination / f"dh_{part}.{suffix}", facecolor="white", **kwargs)
    plt.close(fig)


def generate_diagrams(output_dir=None):
    destination = Path(output_dir) if output_dir is not None else HERE / "report" / "assets"
    destination.mkdir(parents=True, exist_ok=True)
    report = validate_geometry()
    with plt.rc_context({"font.family": "DejaVu Sans", "pdf.fonttype": 42,
                         "ps.fonttype": 42, "font.size": 8}):
        for part in ("left", "right", "head"):
            draw(part, destination)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    print(json.dumps(generate_diagrams(args.output_dir), indent=2))

