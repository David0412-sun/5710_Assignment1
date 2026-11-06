"""View the original robot at zero joint angles with a fixed base.

On macOS, run from the repository root:
    conda activate open_duck
    mjpython -m assignments.assignment_1_kinematics.view_model

Drag to rotate/pan and scroll to zoom. Close the window to exit.
"""
import os
import sys
import time

import mujoco
import mujoco.viewer

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from playground.open_duck_mini_v2 import constants
from playground.open_duck_mini_v2.mujoco_infer_base import MJInferBase


def main():
    simulator = MJInferBase(constants.FLAT_TERRAIN_XML.as_posix())
    model, data = simulator.model, simulator.data
    shell_meshes = {
        "head", "body_front", "body_back", "body_middle_bottom",
        "body_middle_top", "left_cache", "right_cache",
        "antenna", "left_antenna_holder", "right_antenna_holder",
        "left_roll_to_pitch", "right_roll_to_pitch",
    }
    for gid in range(model.ngeom):
        if model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh_name = model.mesh(int(model.geom_dataid[gid])).name
            if mesh_name in shell_meshes:
                model.geom_group[gid] = 5  # Viewer-only shell visibility group.
    data.qpos[:] = model.qpos0  # All hinge angles zero; base at the XML pose.
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    base_joint = model.joint("floating_base").id
    model.jnt_group[base_joint] = 5  # Replace the large default free-joint glyph.

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0, 0, 0.25]
        viewer.cam.distance = 0.85
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -15
        viewer.opt.geomgroup[3] = 0  # Hide duplicate collision meshes.
        viewer.opt.geomgroup[5] = 0  # Hide exterior covers; keep structural parts.
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
        viewer.opt.label = mujoco.mjtLabel.mjLABEL_JOINT
        viewer.opt.jointgroup[5] = 0
        with viewer.lock():
            marker = viewer.user_scn.geoms[0]
            mujoco.mjv_initGeom(
                marker, mujoco.mjtGeom.mjGEOM_SPHERE,
                [0.008, 0.008, 0.008], data.xanchor[base_joint],
                [1, 0, 0, 0, 1, 0, 0, 0, 1], [0.1, 0.6, 0.9, 1],
            )
            marker.label = "base"
            viewer.user_scn.ngeom = 1
        while viewer.is_running():
            # No mj_step: the base and joints remain fixed for screenshots.
            viewer.sync()
            time.sleep(1 / 60)


if __name__ == "__main__":
    main()
