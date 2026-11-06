"""Headless regression checks using MuJoCo as an independent pose oracle.

From the repository root:
    .venv/bin/python -m unittest assignments.assignment_1_kinematics.test_kinematics -v
"""
from contextlib import redirect_stdout
import io
from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from assignments.assignment_1_kinematics.forward_kinematics import (
    forward_kinematics, joint_names, parameters, site_pose,
)
from assignments.assignment_1_kinematics.inverse_kinematics import (
    inverse_kinematics, objective,
)
from playground.open_duck_mini_v2.constants import FLAT_TERRAIN_XML
from playground.open_duck_mini_v2.mujoco_infer_base import MJInferBase


PARTS = ("left", "right", "head")
INITIAL_Q = {
    "left": np.array([0, 0, -0.4, 0.8, -0.4]),
    "right": np.array([0, 0, 0.4, 0.8, -0.4]),
    "head": np.array([0.2, -0.4, 0, 0]),
}


def oracle_pose(model, data, part):
    """Compute base^-1 @ site from MuJoCo's world-frame transforms."""
    base = model.body("base").id
    site = model.site("head" if part == "head" else f"{part}_foot").id
    world_base = np.eye(4)
    world_base[:3, :3] = data.xmat[base].reshape(3, 3)
    world_base[:3, 3] = data.xpos[base]
    world_site = np.eye(4)
    world_site[:3, :3] = data.site_xmat[site].reshape(3, 3)
    world_site[:3, 3] = data.site_xpos[site]
    return np.linalg.solve(world_base, world_site)


def chain_addresses(model, part):
    ids = np.array([model.joint(name).id for name in joint_names(part)])
    return model.jnt_qposadr[ids], model.jnt_range[ids]


class KinematicsRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Load the actual XML directly, independently of the DH model.
        cls.model = mujoco.MjModel.from_xml_path(str(FLAT_TERRAIN_XML))

    def setUp(self):
        self.data = mujoco.MjData(self.model)

    def assert_pose_matches(self, actual, expected):
        self.assertLess(np.linalg.norm(actual[:3, 3] - expected[:3, 3]), 1e-6)
        angle = Rotation.from_matrix(
            actual[:3, :3].T @ expected[:3, :3]).magnitude()
        self.assertLess(angle, 1e-5)

    def test_zero_pose_matches_xml_and_dimensions(self):
        zero_positions = {
            "left": [-0.03456, 0.0894, -0.1935991],
            "right": [-0.03456, -0.0918, -0.1935991],
            "head": [0.006, 0.0001, 0.2363009],
        }
        mujoco.mj_forward(self.model, self.data)
        for part in PARTS:
            with self.subTest(part=part):
                expected = oracle_pose(self.model, self.data, part)
                np.testing.assert_allclose(expected[:3, 3], zero_positions[part],
                                           atol=1e-10, rtol=0)
                np.testing.assert_allclose(expected[:3, :3], np.eye(3),
                                           atol=1e-10, rtol=0)
                actual = forward_kinematics(np.zeros(len(joint_names(part))),
                                            *parameters(part))
                self.assert_pose_matches(actual, expected)

    def test_random_configurations_with_translated_and_rotated_base(self):
        rng = np.random.default_rng(5710)
        base_addr = int(self.model.jnt_qposadr[self.model.joint("floating_base").id])
        quaternion = Rotation.from_euler("xyz", [0.31, -0.29, 0.62]).as_quat()
        moved_base = np.r_[[0.17, -0.2, 0.6], quaternion[[3, 0, 1, 2]]]
        for sample in range(12):
            for part in PARTS:
                addresses, limits = chain_addresses(self.model, part)
                self.data.qpos[addresses] = rng.uniform(limits[:, 0], limits[:, 1])
            mujoco.mj_forward(self.model, self.data)
            original = {part: oracle_pose(self.model, self.data, part)
                        for part in PARTS}
            self.data.qpos[base_addr:base_addr + 7] = moved_base
            mujoco.mj_forward(self.model, self.data)
            for part in PARTS:
                with self.subTest(sample=sample, part=part):
                    expected = oracle_pose(self.model, self.data, part)
                    np.testing.assert_allclose(expected, original[part],
                                               atol=1e-12, rtol=0)
                    addresses, _ = chain_addresses(self.model, part)
                    actual = forward_kinematics(self.data.qpos[addresses],
                                                *parameters(part))
                    self.assert_pose_matches(actual, expected)
                    # Exercise the production world-to-base conversion too.
                    env = type("PoseState", (), {"model": self.model,
                                                "data": self.data})()
                    np.testing.assert_allclose(site_pose(env, part), expected,
                                               atol=1e-12, rtol=0)
            self.data.qpos[base_addr:base_addr + 7] = self.model.qpos0[
                base_addr:base_addr + 7]

    def test_objective_uses_metric_weight_and_rotation_angle(self):
        displacement = np.array([0.003, -0.004, 0.005])
        for part in PARTS:
            table, signs, base = parameters(part)
            q = INITIAL_Q[part].copy()
            pose = forward_kinematics(q, table, signs, base)
            self.assertLess(objective(q, pose, table, signs, base), 1e-24)
            # A noncommuting rotation about several axes is not an Euler error.
            for rotvec in (np.array([0.2, -0.3, 0.4]),
                           np.array([2.9, -0.2, 0.1])):
                with self.subTest(part=part, rotvec=rotvec):
                    target = pose.copy()
                    target[:3, 3] += displacement
                    target[:3, :3] = (Rotation.from_rotvec(rotvec).as_matrix()
                                         @ pose[:3, :3])
                    expected = 100 * displacement.dot(displacement) + rotvec.dot(rotvec)
                    self.assertAlmostEqual(objective(q, target, table, signs, base),
                                           expected, places=11)
                    np.testing.assert_array_equal(q, INITIAL_Q[part])


class InverseKinematicsRegressionTests(unittest.TestCase):
    def setUp(self):
        with redirect_stdout(io.StringIO()):
            self.env = MJInferBase(FLAT_TERRAIN_XML)
        for part in PARTS:
            addresses, _ = chain_addresses(self.env.model, part)
            self.env.data.qpos[addresses] = INITIAL_Q[part]
        mujoco.mj_forward(self.env.model, self.env.data)

    def assert_selected_joints_only_and_within_limits(self, part, before):
        addresses, limits = chain_addresses(self.env.model, part)
        unchanged = np.ones(len(before), dtype=bool)
        unchanged[addresses] = False
        np.testing.assert_array_equal(self.env.data.qpos[unchanged], before[unchanged])
        solved = self.env.data.qpos[addresses]
        self.assertTrue(np.all(solved >= limits[:, 0]))
        self.assertTrue(np.all(solved <= limits[:, 1]))

    def test_current_pose_is_a_no_op(self):
        for part in PARTS:
            with self.subTest(part=part):
                before = self.env.data.qpos.copy()
                target = oracle_pose(self.env.model, self.env.data, part)
                position_error, orientation_error = inverse_kinematics(
                    target, part, self.env)
                self.assertLess(position_error, 1e-10)
                self.assertLess(orientation_error, 1e-10)
                np.testing.assert_array_equal(self.env.data.qpos, before)

    def test_reachable_nearby_poses_and_other_chain_preservation(self):
        for part in PARTS:
            with self.subTest(part=part):
                before = self.env.data.qpos.copy()
                addresses, _ = chain_addresses(self.env.model, part)
                goal_data = mujoco.MjData(self.env.model)
                goal_data.qpos[:] = before
                delta = np.array([0.002, -0.002, 0.008, -0.003, 0.002])[:len(addresses)]
                goal_data.qpos[addresses] += delta
                mujoco.mj_forward(self.env.model, goal_data)
                target = oracle_pose(self.env.model, goal_data, part)
                position_error, orientation_error = inverse_kinematics(
                    target, part, self.env)
                self.assertLess(position_error, 1e-5)
                self.assertLess(orientation_error, 1e-4)
                self.assert_selected_joints_only_and_within_limits(part, before)

    def test_large_step_is_projected_to_joint_limits(self):
        part = "head"
        before = self.env.data.qpos.copy()
        target = oracle_pose(self.env.model, self.env.data, part)
        target[:3, 3] += [10, 10, 10]  # Deliberately unreachable target.
        with redirect_stdout(io.StringIO()):
            inverse_kinematics(target, part, self.env,
                               max_iterations=1, alpha=1e6)
        self.assert_selected_joints_only_and_within_limits(part, before)
        addresses, limits = chain_addresses(self.env.model, part)
        solved = self.env.data.qpos[addresses]
        self.assertTrue(np.any(np.isclose(solved, limits[:, 0])
                               | np.isclose(solved, limits[:, 1])))


if __name__ == "__main__":
    unittest.main()
