"""Actual plant/MuJoCo integration; requires sourced ROS workspace, no viewer."""
import os
from unittest.mock import patch

import mujoco
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('minitrone_interfaces.msg')
from minitrone_plant.plant import PlantRosNode


@pytest.fixture
def plant_factory(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    nodes = []

    def create(enabled=True, case='B', extra=()):
        # ROS graph is real; only the background thread is withheld so that
        # inputs, sample time and noise can be compared at identical sim ticks.
        for previous in nodes:
            previous.destroy_node()
        nodes.clear()
        if rclpy.ok():
            rclpy.shutdown()
        rclpy.init(args=['--ros-args', '-p', 'enable_viewer:=false', '-p',
                         f'enable_high_fidelity:={str(enabled).lower()}', '-p',
                         f'contact_test_case:={case}', *extra])
        with patch('threading.Thread.start'):
            node = PlantRosNode()
        nodes.append(node)
        return node

    yield create
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


@pytest.mark.parametrize('enabled', [False, True])
@pytest.mark.parametrize('case', ['A', 'B', 'C', 'D'])
def test_cases_run_finite_with_existing_publishers(plant_factory, enabled, case):
    node = plant_factory(enabled, case)
    node.ctrl_recv = np.r_[np.full(4, 15.0), np.zeros(4)]
    for _ in range(800):
        node._physics_step()
        node._publish_state()
    assert node.data.time == pytest.approx(2.0)
    assert np.isfinite(node.data.qpos).all()
    assert not np.any(node.data.warning.number)
    expected = {'A': 1, 'B': 3, 'C': 6}
    if case in expected:
        assert node.model.geom_condim[node.wall_geom_id] == expected[case]


def test_servo_reference_and_actual_hinge_lag(plant_factory):
    node = plant_factory(extra=('-p', 'enable_residual_wrench:=false'))
    node.ctrl_recv = np.r_[np.zeros(4), np.full(4, 0.4)]
    node._physics_step()
    actual = np.array([node._sensing(s)[0] for s in node.sid_servo_ang])
    np.testing.assert_allclose(actual, 0, atol=1e-8)
    for _ in range(19):
        node._physics_step()
    actual = np.array([node._sensing(s)[0] for s in node.sid_servo_ang])
    assert np.all(actual > 0)
    assert np.all(actual < 0.4)
    assert np.all(node.high_fidelity.servo.value < 0.4)


def test_residual_is_external_and_adds_to_explicit_command(plant_factory):
    node = plant_factory()
    node.ctrl_recv = np.zeros(8)
    node.external_force_body[:] = [1, 2, 3]
    node.external_moment_body[:] = [0.1, 0.2, 0.3]
    node._apply_external_wrench()
    rotation = node.data.xmat[node.base_body_id].reshape(3, 3)
    residual = node.high_fidelity.residual.value
    np.testing.assert_allclose(node.data.xfrc_applied[node.base_body_id, :3], rotation @ (node.external_force_body + residual[:3]))
    np.testing.assert_allclose(node.data.xfrc_applied[node.base_body_id, 3:], rotation @ (node.external_moment_body + residual[3:]))
    mujoco.mj_forward(node.model, node.data)
    force, moment = node._actuation_wrench_body()
    np.testing.assert_allclose(force, 0)
    np.testing.assert_allclose(moment, 0)
    assert np.linalg.norm(node.data.xfrc_applied[node.base_body_id]) > 0


def test_identical_seed_plant_trajectory_and_sensor_sequence(plant_factory):
    outputs = []
    for _ in range(2):
        node = plant_factory()
        node.ctrl_recv = np.r_[np.full(4, 17.0), [0.1, -0.1, 0.1, -0.1]]
        trajectory = []
        for _ in range(100):
            node._physics_step()
            trajectory.append(np.concatenate([node.data.qpos.copy(), *node._measure_state()]))
        outputs.append(trajectory)
    np.testing.assert_array_equal(outputs[0], outputs[1])


@pytest.mark.parametrize('case', ['A', 'B', 'C', 'D'])
def test_actual_plate_contact_with_com_uncertainty(plant_factory, case):
    node = plant_factory(case=case)
    # Put the +X face 1 mm inside the palm near its center, off the floor.
    root = int(node.model.joint('root').qposadr[0])
    node.data.qpos[root:root + 3] = [1.166, 0.0, 1.0]
    mujoco.mj_forward(node.model, node.data)
    node._physics_step()
    from unittest.mock import Mock
    node.pub_contact_wrench_gt = Mock()
    node.pub_contact_wrench_center_gt = Mock()
    node.pub_contact_count = Mock()
    counts = []
    # ROS serializes on publish; capture by value because the old implementation
    # reuses this Int32 object for the subsequent other-contact publication.
    node.pub_contact_count.publish.side_effect = lambda msg: counts.append(msg.data)
    node._plate_wall_contact_debug()
    assert counts[-1] > 0
    com = node.pub_contact_wrench_gt.publish.call_args.args[0]
    center = node.pub_contact_wrench_center_gt.publish.call_args.args[0]
    assert np.linalg.norm(com.force) > 0
    # Wrench shift must be invariant to where the inertial origin was sampled.
    lever = node.contact_center_body - node.model.body_ipos[node.base_body_id]
    np.testing.assert_allclose(
        np.asarray(center.moment), np.asarray(com.moment) - np.cross(lever, com.force),
        atol=1e-5, rtol=1e-5)
    assert not np.any(node.data.warning.number)


def test_com_offset_changes_physics_not_tracker_location(plant_factory):
    node = plant_factory(extra=('-p', 'com_random_range:=0.0', '-p',
                                'com_offset:=[0.001, 0.002, 0.003]', '-p',
                                'enable_sensor_model:=false'))
    node.data.qvel[3:6] = [0, 0, 1]
    mujoco.mj_forward(node.model, node.data)
    # Pure rotation about the body origin moves CoM, but not the tracker there.
    assert np.linalg.norm(node._sensing(node.sid_vel)) > 0
    with patch.object(node, '_noisy', side_effect=lambda x, sigma: x):
        position, velocity, *_ = node._measure_state()
    np.testing.assert_allclose(position, node.data.xpos[node.base_body_id])
    np.testing.assert_allclose(velocity, [0, 0, 0], atol=1e-12)
