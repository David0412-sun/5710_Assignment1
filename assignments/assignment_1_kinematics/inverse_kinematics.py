"""Student gradient-descent IK template: complete TODOs 1-3 after the FK TODOs."""
import os
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from assignments.assignment_1_kinematics.forward_kinematics import (
    forward_kinematics, joint_names, parameters, site_pose,
)

from playground.open_duck_mini_v2.mujoco_infer_base import MJInferBase
from playground.open_duck_mini_v2.constants import FLAT_TERRAIN_XML


def objective(q, target, dh_table, signs, T_base_0):
    """Return the weighted squared pose error for joint angles q (radians).

    target is a base-frame 4x4 pose with translation in metres.
    Candidate poses come from FK; this function does not change MuJoCo's state.
    """
    position_scale = 10.0  # Inverse metres; scales position relative to rotation.

    # TODO 1: Predict the site pose with forward_kinematics(). Compute the
    # position error and the orientation error as a rotation vector.
    # Return sum((position_error * position_scale)**2) + sum(rotation_error**2).
    raise NotImplementedError("IK TODO 1: pose-error objective")


def inverse_kinematics(target, part, env, max_iterations=5000, alpha=0.1):
    """Move one site toward a base-frame target pose using gradient descent.

    target: 4x4 pose, with translation in metres.
    part: "left", "right", or "head"; env: the existing MJInferBase environment.
    alpha: fixed learning rate for all iterations.
    Update the selected joints and return position/orientation errors (m, rad).
    """
    # 1. Select the chain and use its current joint angles as the initial guess.
    dh_table, signs, T_base_0 = parameters(part)
    indices = [env.actuator_names.index(name) for name in joint_names(part)]
    joint_ids = np.asarray(env.actuator_joint_ids)[indices]
    limits = env.model.jnt_range[joint_ids]
    angles = env.get_actuator_joints_qpos(env.data.qpos)
    q = angles[indices].copy()

    # 2. Evaluate the initial pose error using the shared cost function.
    cost = objective(q, target, dh_table, signs, T_base_0)

    # 3. Repeatedly estimate the gradient and update the joint angles.
    h = 1e-6  # Joint-angle perturbation in radians for finite differences.
    for _ in range(max_iterations):
        # 3a. Stop when the cost guarantees errors below 1e-5 m and 1e-4 rad.
        if cost < 1e-8:
            break

        # 3b. Perturb one joint at a time to estimate d(cost)/d(q_i).
        gradient = np.zeros(len(q))
        for i in range(len(q)):
            # TODO 2: Copy q into q_plus and q_minus. Change only joint i by
            # +h and -h, evaluate both costs, and store their central difference
            # (cost_plus - cost_minus) / (2 * h) in gradient[i].
            raise NotImplementedError("IK TODO 2: finite-difference gradient")

        # 3c. Take a fixed-size gradient step and enforce the joint limits.
        # TODO 3: Update q using q - alpha * gradient. Clip each joint between
        # its lower limit limits[:, 0] and upper limit limits[:, 1].
        raise NotImplementedError("IK TODO 3: bounded gradient-descent update")
        cost = objective(q, target, dh_table, signs, T_base_0)

    # 4. Write the solved angles once and update MuJoCo's kinematics.
    angles[indices] = q
    env.set_actuator_joints_qpos(angles, env.data.qpos)
    mujoco.mj_forward(env.model, env.data)

    # 5. Compare the measured site pose with the target, including orientation.
    current = site_pose(env, part)
    position_error = np.linalg.norm(target[:3, 3] - current[:3, 3])
    orientation_error = Rotation.from_matrix(
        target[:3, :3] @ current[:3, :3].T).magnitude()
    if position_error >= 1e-5 or orientation_error >= 1e-4:
        print(f"{part}: target tolerance not reached: "
              f"position={position_error:.3e} m, orientation={orientation_error:.3e} rad")
    return position_error, orientation_error


def left_foot_trajectory(start, phase):
    """Move forward 20 mm and up 20 mm, then return with fixed orientation."""
    target = start.copy()
    target[0, 3] += 0.020 * np.sin(np.pi * phase)**2
    target[2, 3] += 0.020 * np.sin(np.pi * phase)**2
    return target


def right_foot_trajectory(start, phase):
    """Move forward 20 mm and up 20 mm, then return with fixed orientation."""
    target = start.copy()
    target[0, 3] += 0.020 * np.cos(np.pi * phase)**2
    target[2, 3] += 0.020 * np.cos(np.pi * phase)**2
    return target


def head_trajectory(start, phase):
    """Nod 0.1 rad about the neck axis and return (position AND orientation)."""
    # A neck-centred arc gives a reachable full-pose task for the 4-DOF head.
    pivot = np.array([0.001, 0.01915, 0.0900009])  # Neck anchor in base coordinates.
    angle = 0.1 * np.sin(np.pi * phase)**2
    R = Rotation.from_rotvec([0, -angle, 0]).as_matrix()
    target = start.copy()
    target[:3, 3] = pivot + R @ (start[:3, 3] - pivot)
    target[:3, :3] = R @ start[:3, :3]
    return target


def run(parts, cycles=5):
    """Repeat the selected trajectories with the base fixed."""
    env = MJInferBase(FLAT_TERRAIN_XML)
    # Keep the soles above the floor for the bent-leg starting configuration.
    base_pose = env.get_floating_base_qpos(env.data.qpos).copy()
    base_pose[2] = 0.19
    env.set_floating_base_qpos(base_pose, env.data.qpos)
    initial_q = {
        "left": [0, 0, -0.4, 0.8, -0.4],
        "right": [0, 0, 0.4, 0.8, -0.4],
        "head": [0.2, -0.4, 0, 0],
    }
    trajectories = {"left": left_foot_trajectory,
                    "right": right_foot_trajectory, 
                    "head": head_trajectory}
    
    angles = env.get_actuator_joints_qpos(env.data.qpos)
    for part in parts:
        indices = [env.actuator_names.index(name) for name in joint_names(part)]
        angles[indices] = initial_q[part]
    env.set_actuator_joints_qpos(angles, env.data.qpos)
    mujoco.mj_forward(env.model, env.data)

    starts = {part: site_pose(env, part) for part in parts}
    max_errors = {part: np.zeros(2) for part in parts}

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        viewer.cam.lookat[:] = [0, 0, 0.25]
        viewer.cam.distance = 0.85
        for cycle in range(cycles):
            for phase in np.linspace(0, 1, 101):
                if not viewer.is_running():
                    return
                with viewer.lock():
                    for part in parts:
                        target = trajectories[part](starts[part], phase)
                        errors = inverse_kinematics(target, part, env)
                        max_errors[part] = np.maximum(max_errors[part], errors)
                viewer.sync()
                time.sleep(0.03)

    for part in parts:
        position_error, orientation_error = max_errors[part]
        print(f"{part}: {cycles} cycles, max position={position_error * 1000:.6f} mm, "
              f"max orientation={orientation_error:.3e} rad")


if __name__ == "__main__":
    run(("left", "right", "head"))
