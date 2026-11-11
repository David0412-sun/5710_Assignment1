"""Render report illustrations from real FK/IK states without changing results.

Run once to solve the first half-cycle and render; --reuse-state only redraws
the previously saved illustrative state after checking its source hashes.
These illustrations are separate from the complete recorded experiment.
"""
from __future__ import annotations

import argparse
from contextlib import redirect_stdout
import hashlib
import io
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
ASSETS = HERE / "report" / "assets"
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".tmp" / "matplotlib"))
sys.path.insert(0, str(ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import mujoco
import numpy as np

from assignments.assignment_1_kinematics.forward_kinematics import (
    forward_kinematics, joint_names, parameters, site_pose,
)
from assignments.assignment_1_kinematics.inverse_kinematics import (
    inverse_kinematics, left_foot_trajectory, right_foot_trajectory, head_trajectory,
)
from playground.open_duck_mini_v2.constants import FLAT_TERRAIN_XML
from playground.open_duck_mini_v2.mujoco_infer_base import MJInferBase

PARTS = ("left", "right", "head")
COLORS = {"left": "#00759a", "right": "#aa3574", "head": "#7046a1"}
NAMES = {"left": "Left foot", "right": "Right foot", "head": "Head site"}
TRAJECTORIES = {"left": left_foot_trajectory, "right": right_foot_trajectory,
                "head": head_trajectory}
INITIAL_Q = {"left": [0, 0, -0.4, 0.8, -0.4],
             "right": [0, 0, 0.4, 0.8, -0.4], "head": [0.2, -0.4, 0, 0]}
WIDTH, HEIGHT = 900, 1200


def source_hashes():
    paths = [HERE / "forward_kinematics.py", HERE / "inverse_kinematics.py",
             ROOT / "playground/open_duck_mini_v2/mujoco_infer_base.py",
             Path(FLAT_TERRAIN_XML), Path(FLAT_TERRAIN_XML).with_name("open_duck_mini_v2.xml")]
    return {path.relative_to(ROOT).as_posix(): hashlib.sha256(
        path.read_bytes().replace(b"\r\n", b"\n")).hexdigest() for path in paths}


def measure_states():
    with redirect_stdout(io.StringIO()):
        env = MJInferBase(FLAT_TERRAIN_XML)
    zero_data = mujoco.MjData(env.model)
    mujoco.mj_forward(env.model, zero_data)
    zero_env = type("State", (), {"model": env.model, "data": zero_data})()
    fk_poses = {}
    for part in PARTS:
        pose = forward_kinematics(np.zeros(len(joint_names(part))), *parameters(part))
        measured = site_pose(zero_env, part)
        if not np.allclose(pose, measured, rtol=0, atol=1e-10):
            raise RuntimeError(f"Zero-pose FK mismatch: {part}")
        fk_poses[part] = pose.tolist()

    base = env.get_floating_base_qpos(env.data.qpos).copy()
    base[2] = 0.19
    env.set_floating_base_qpos(base, env.data.qpos)
    angles = env.get_actuator_joints_qpos(env.data.qpos)
    for part in PARTS:
        indices = [env.actuator_names.index(name) for name in joint_names(part)]
        angles[indices] = INITIAL_Q[part]
    env.set_actuator_joints_qpos(angles, env.data.qpos)
    mujoco.mj_forward(env.model, env.data)
    start_qpos = env.data.qpos.copy()
    starts = {part: site_pose(env, part) for part in PARTS}
    final_targets = {part: TRAJECTORIES[part](starts[part], 0.5) for part in PARTS}
    errors = {}
    for phase in np.linspace(0, 1, 101)[:51]:
        for part in PARTS:
            target = TRAJECTORIES[part](starts[part], phase)
            position, orientation = inverse_kinematics(target, part, env)
            if not (position < 1e-5 and orientation < 1e-4):
                raise RuntimeError(f"Illustrative IK sample failed: {phase}, {part}")
            errors[part] = {"position_error_m": float(position),
                            "orientation_error_rad": float(orientation)}
    return {
        "description": "Illustrative zero-pose FK and first half of cycle 1; not a replacement for full results",
        "source_sha256_lf": source_hashes(),
        "fk": {"qpos": zero_data.qpos.tolist(), "base_frame_site_poses": fk_poses},
        "ik": {"phase": 0.5, "phases_solved": 51, "initial_joint_angles_rad": INITIAL_Q,
               "start_qpos": start_qpos.tolist(), "solved_qpos": env.data.qpos.tolist(),
               "start_poses": {p: pose.tolist() for p, pose in starts.items()},
               "target_poses": {p: pose.tolist() for p, pose in final_targets.items()},
               "solved_poses": {p: site_pose(env, p).tolist() for p in PARTS},
               "final_errors": errors},
        "image_notes": {
            "projection": "True perspective projection from MuJoCo's render camera",
            "overlay": "Colored circles: actual sites; orange diamonds: phase 0.5 targets",
            "base_axes": "Red x, green y, blue z; dashed arrows overlay the true base origin",
            "pose": "Robot left/right are physical chain names, not image sides",
        },
    }


def world_pose(model, data, pose):
    base_id = model.body("base").id
    rotation = data.xmat[base_id].reshape(3, 3)
    return data.xpos[base_id] + rotation @ np.asarray(pose)[:3, 3]


def render(model, data):
    # Hide only the floor in the illustration. Geometry and qpos are unchanged.
    model.geom_group[model.geom("floor").id] = 5
    model.vis.global_.offwidth = WIDTH
    model.vis.global_.offheight = HEIGHT
    model.vis.headlight.ambient[:] = [0.55, 0.55, 0.55]
    model.vis.headlight.diffuse[:] = [0.7, 0.7, 0.7]
    options = mujoco.MjvOption()
    options.geomgroup[3] = 0  # Hide duplicate collision meshes.
    options.geomgroup[5] = 0
    options.sitegroup[:] = 0
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [-0.01, 0, 0.265]
    camera.distance = 0.94
    camera.azimuth = 45
    camera.elevation = -13
    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        renderer.update_scene(data, camera=camera, scene_option=options)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        pixels = renderer.render()
        gl = renderer.scene.camera
        position = (np.asarray(gl[0].pos) + np.asarray(gl[1].pos)) / 2
        forward = np.asarray(gl[0].forward).copy()
        up = np.asarray(gl[0].up).copy()
        near = gl[0].frustum_near
        focal = HEIGHT * near / (gl[0].frustum_top - gl[0].frustum_bottom)
    right = np.cross(forward, up)

    def project(points):
        delta = np.atleast_2d(points) - position
        depth = delta @ forward
        if np.any(depth <= 0):
            raise ValueError("Point lies behind the illustration camera")
        return np.column_stack([WIDTH / 2 + focal * (delta @ right) / depth,
                                HEIGHT / 2 - focal * (delta @ up) / depth])
    return pixels, project


def annotate(model, data, path, title, targets=None, starts=None):
    pixels, project = render(model, data)
    fig = plt.figure(figsize=(6, 8), dpi=150, facecolor="white")
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(pixels)
    ax.set_xlim(0, WIDTH)
    ax.set_ylim(HEIGHT, 0)
    ax.axis("off")
    ax.text(WIDTH / 2, 45, title, ha="center", va="center", fontsize=19,
            fontweight="bold", color="#26313b")
    base_id = model.body("base").id
    base = data.xpos[base_id]
    rotation = data.xmat[base_id].reshape(3, 3)
    base_uv = project(base)[0]
    if targets is None:
        for axis, color, letter in zip(range(3), ["#bf3b35", "#168552", "#285eb5"], "xyz"):
            end = project(base + 0.058 * rotation[:, axis])[0]
            ax.annotate("", xy=end, xytext=base_uv,
                        arrowprops={"arrowstyle": "-|>", "lw": 2.4, "color": color,
                                    "linestyle": "--", "mutation_scale": 16})
            ax.text(end[0], end[1] - 9, letter, color=color, fontsize=17,
                    ha="center", va="bottom", fontweight="bold")
        ax.annotate("Base B", xy=base_uv, xytext=(50, 585), fontsize=17,
                    ha="left", va="center", color="#26313b",
                    bbox={"facecolor": "white", "edgecolor": "none", "alpha": .88, "pad": 2},
                    arrowprops={"arrowstyle": "-", "color": "#52606d", "lw": 1.5})
    positions = {}
    for part in PARTS:
        site = model.site("head" if part == "head" else f"{part}_foot").id
        uv = project(data.site_xpos[site])[0]
        positions[part] = uv
        if targets is not None:
            target_uv = project(world_pose(model, data, targets[part]))[0]
            ax.scatter(*target_uv, s=210, marker="D", facecolors="none",
                       edgecolors="#df8a11", linewidths=2.2, zorder=5)
            curve = [world_pose(model, data, TRAJECTORIES[part](starts[part], phase))
                     for phase in np.linspace(0, .5, 60)]
            curve_uv = project(curve)
            ax.plot(*curve_uv.T, color="#df8a11", linestyle="--", lw=1.7, zorder=4)
        ax.scatter(*uv, s=74, color=COLORS[part], edgecolors="white", linewidths=1.0, zorder=6)

    feet = sorted(("left", "right"), key=lambda p: positions[p][0])
    text_positions = {feet[0]: (40, 990), feet[1]: (860, 990), "head": (860, 290)}
    for part in PARTS:
        text_pos = text_positions[part]
        ax.annotate(NAMES[part], xy=positions[part], xytext=text_pos,
                    ha="left" if text_pos[0] < WIDTH / 2 else "right", va="center",
                    fontsize=17, color=COLORS[part],
                    bbox={"facecolor": "white", "edgecolor": "none", "alpha": .9, "pad": 3},
                    arrowprops={"arrowstyle": "-", "color": COLORS[part], "lw": 1.5}, zorder=8)
    if targets is None:
        ax.text(WIDTH / 2, 1125, "All joint angles = 0 rad", ha="center", fontsize=16,
                color="#4a535c")
    else:
        handles = [Line2D([0], [0], marker="o", color="none", markerfacecolor="#536878",
                          markeredgecolor="white", markersize=7, label="Actual site"),
                   Line2D([0], [0], marker="D", color="none", markerfacecolor="none",
                          markeredgecolor="#df8a11", markersize=9, label="Target at phase 0.5")]
        ax.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, .02),
                  ncol=1, frameon=False, fontsize=15, labelspacing=.4)
    fig.savefig(path, dpi=150, facecolor="white")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reuse-state", action="store_true")
    args = parser.parse_args()
    ASSETS.mkdir(parents=True, exist_ok=True)
    metadata_path = ASSETS / "illustration_metadata.json"
    if args.reuse_state:
        state = json.loads(metadata_path.read_text(encoding="utf-8"))
        if state["source_sha256_lf"] != source_hashes():
            raise RuntimeError("Illustration sources changed; rerun without --reuse-state")
    else:
        state = measure_states()
        metadata_path.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")
    model = mujoco.MjModel.from_xml_path(str(FLAT_TERRAIN_XML))
    data = mujoco.MjData(model)
    data.qpos[:] = state["fk"]["qpos"]
    mujoco.mj_forward(model, data)
    annotate(model, data, ASSETS / "fk_robot.png", "Forward kinematics")
    starts = {part: np.array(pose) for part, pose in state["ik"]["start_poses"].items()}
    targets = {part: np.array(pose) for part, pose in state["ik"]["target_poses"].items()}
    for key, filename, title in (("start_qpos", "ik_start.png", "IK: initial pose"),
                                 ("solved_qpos", "ik_solved.png", "IK: solved at phase 0.5")):
        data.qpos[:] = state["ik"][key]
        mujoco.mj_forward(model, data)
        annotate(model, data, ASSETS / filename, title, targets, starts)
    print("Created fk_robot.png, ik_start.png, ik_solved.png and illustration_metadata.json")
    print(json.dumps(state["ik"]["final_errors"], indent=2))


if __name__ == "__main__":
    main()
