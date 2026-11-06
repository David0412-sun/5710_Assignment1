"""Student FK template: complete TODOs 1-7, then run this file to check each site."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

def rotation_matrix_x(alpha):
    """Return a 4x4 rotation about x by alpha radians."""
    c, s = np.cos(alpha), np.sin(alpha)
    return np.array([[1, 0, 0, 0], 
                     [0, c, -s, 0],
                     [0, s, c, 0], 
                     [0, 0, 0, 1]])

def translation_matrix_x(x):
    """Return a 4x4 translation along x by x metres."""
    return np.array([[1, 0, 0, x], 
                     [0, 1, 0, 0],
                     [0, 0, 1, 0], 
                     [0, 0, 0, 1]])

def rotation_matrix_z(theta):
    """Return a 4x4 rotation about z by theta radians."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0, 0], 
                     [s, c, 0, 0],
                     [0, 0, 1, 0], 
                     [0, 0, 0, 1]])

def translation_matrix_z(z):
    """Return a 4x4 translation along z by z metres."""
    return np.array([[1, 0, 0, 0], 
                     [0, 1, 0, 0],
                     [0, 0, 1, z], 
                     [0, 0, 0, 1]])

def forward_kinematics(q, dh_table, signs, T_base_0):
    """Return the 4x4 site pose in the base frame (translation in metres).

    The final DH row is fixed and places the last frame at the site.
    """
    T_list = forward_kinematics_transforms(q, dh_table, signs, T_base_0)
    return T_list[-1]

def forward_kinematics_transforms(q, dh_table, signs, T_base_0):
    """Return one base-frame 4x4 pose per DH row, including the site.

    The first len(q) rows are joints; the last row is the fixed site row.
    Each row is [alpha_previous, a_previous, theta_offset, d] (rad, m).
    """
    T_list = []
    T = T_base_0.copy()

    for i, (alpha, a, theta_offset, d) in enumerate(dh_table):
        theta = signs[i] * q[i] + theta_offset if i < len(q) else theta_offset

        # TODO 1: Form A_i = Rx(alpha) Tx(a) Rz(theta) Tz(d).
        # Update T = T @ A_i and append the resulting T_base_i to T_list.

    return T_list

def right_leg_parameters():
    """
        Return the right-leg DH table, joint signs, and base-to-0 transform.
    """
    beta = np.arctan2(19, 35)

    # T_base_0 is the transform from the base frame to the frame 0, which needs to be defined in the DH table. 
    # z_0 is the axis of the base frame, and z_1 is the axis of the first joint.
    # The angle beta can be deined in different ways, you can modify it as you wish, but it should be consistent with the DH table.
    T_base_0 = rotation_matrix_z(-np.pi / 2 - beta)

    # TODO 2: Define dh_table with five joint rows and one fixed site row.
    # Each row is [alpha, a, theta_offset, d] in radians and metres.

    # TODO 3: Define five signs (+1 or -1) so DH theta follows MuJoCo q.

    return dh_table, signs, T_base_0

def left_leg_parameters():
    """
        Return the left-leg DH table, joint signs, and base-to-0 transform.
    """
    beta = np.arctan2(19, 35)

    T_base_0 = rotation_matrix_z(np.pi / 2 + beta)

    # TODO 4: Define dh_table with five joint rows and one fixed site row.
    # Each row is [alpha, a, theta_offset, d] in radians and metres.

    # TODO 5: Define five signs (+1 or -1) so DH theta follows MuJoCo q.

    return dh_table, signs, T_base_0

def head_parameters():
    """
        Return the head DH table, joint signs, and base-to-0 transform.
    """
    T_base_0 = np.eye(4)
    T_base_0[:3, 3] = np.array([1, 19.15, 90.0009]) / 1000

    # TODO 6: Define dh_table with four joint rows and one fixed site row.
    # Each row is [alpha, a, theta_offset, d] in radians and metres.

    # TODO 7: Define four signs (+1 or -1) so DH theta follows MuJoCo q.

    return dh_table, signs, T_base_0

def right_leg_fk(q):
    """Return the right-foot pose in the base frame for five joint angles."""
    dh_table, signs, T_base_0 = right_leg_parameters()
    return forward_kinematics(q, dh_table, signs, T_base_0)

def left_leg_fk(q):
    """Return the left-foot pose in the base frame for five joint angles."""
    dh_table, signs, T_base_0 = left_leg_parameters()
    return forward_kinematics(q, dh_table, signs, T_base_0)

def head_fk(q):
    """Return the head-site pose in the base frame for four joint angles."""
    dh_table, signs, T_base_0 = head_parameters()
    return forward_kinematics(q, dh_table, signs, T_base_0)

def joint_names(part):
    """Return joint names in DH-row order for 'left', 'right', or 'head'."""
    if part == "head":
        return ["neck_pitch", "head_pitch", "head_yaw", "head_roll"]
    return [f"{part}_{name}" for name in
            ("hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle")]

def parameters(part):
    """Return the DH table, joint signs, and base-to-0 transform for part."""
    if part == "left":
        return left_leg_parameters()
    if part == "right":
        return right_leg_parameters()
    return head_parameters()

def site_pose(env, part):
    """Read MuJoCo's 4x4 site pose in the base frame for comparison."""
    model, data = env.model, env.data
    base_id = model.body("base").id
    site_id = model.site("head" if part == "head" else f"{part}_foot").id
    R = data.xmat[base_id].reshape(3, 3).T
    pose = np.eye(4)
    pose[:3, 3] = R @ (data.site_xpos[site_id] - data.xpos[base_id])
    pose[:3, :3] = R @ data.site_xmat[site_id].reshape(3, 3)
    return pose

def validate(part="right"):
    """Compare FK with MuJoCo at 100 seeded random joint configurations.

    Return True when both site position and orientation errors pass.
    Run this file directly to check all three chains after TODOs 1-7.
    """
    import mujoco
    from scipy.spatial.transform import Rotation
    from playground.open_duck_mini_v2.constants import FLAT_TERRAIN_XML
    from playground.open_duck_mini_v2.mujoco_infer_base import MJInferBase

    # Reuse the original environment's model, state and joint mappings.
    env = MJInferBase(FLAT_TERRAIN_XML)
    model, data = env.model, env.data
    # Select the chain; joint order must match the rows of its DH table.
    joints = joint_names(part)
    dh_table, signs, T_base_0 = parameters(part)
    site = "head" if part == "head" else f"{part}_foot"
    count = len(joints)
    print(f"Checking {site}")
    # Select the chain in the environment's actuator order; limits are in radians.
    indices = [env.actuator_names.index(name) for name in joints]
    joint_ids = np.asarray(env.actuator_joint_ids)[indices]
    limits = model.jnt_range[joint_ids]
    # Uniformly sample 100 configurations within limits; seed=0 is reproducible.
    random_q = np.random.default_rng(0).uniform(limits[:, 0], limits[:, 1], (100, count))
    errors = []
    orientation_errors = []
    for q in random_q:
        # Change only this chain through the environment; keep the base fixed.
        angles = env.get_actuator_joints_qpos(data.qpos)
        angles[indices] = q
        env.set_actuator_joints_qpos(angles, data.qpos)
        # Update derived quantities without mj_step; simulation time does not advance.
        mujoco.mj_forward(model, data)
        expected = site_pose(env, part)
        actual = forward_kinematics(q, dh_table, signs, T_base_0)
        # Position error: Euclidean distance between the two site origins, in metres.
        errors.append(np.linalg.norm(actual[:3, 3] - expected[:3, 3]))
        # Orientation error: the axis-angle angle of R_FK.T @ R_MuJoCo.
        # magnitude() returns the minimum rotation angle in [0, pi] rad, not Euler differences.
        orientation_errors.append(Rotation.from_matrix(
            actual[:3, :3].T @ expected[:3, :3]).magnitude())
    # Report maximum errors across samples; convert position to mm for display only.
    print(f"100 random poses: max position={max(errors)*1000:.9f} mm, "
          f"max orientation={max(orientation_errors):.3e} rad")
    # Pass only if all errors are finite, position < 1e-6 m and orientation < 1e-5 rad.
    passed = bool(np.all(np.isfinite(errors)) and max(errors) < 1e-6
                  and np.all(np.isfinite(orientation_errors))
                  and max(orientation_errors) < 1e-5)
    print("PASS" if passed else "FAIL (position <0.001 mm, orientation <1e-5 rad)")
    return passed


if __name__ == "__main__":
    results = [validate(part) for part in ("left", "right", "head")]
    raise SystemExit(0 if all(results) else 1)
