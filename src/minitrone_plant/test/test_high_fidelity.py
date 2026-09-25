"""Behavioral checks for provisional models; no ROS graph/viewer required."""
from pathlib import Path

import mujoco
import numpy as np
import pytest

from minitrone_plant.high_fidelity import (
    ColoredWrenchProcess, DelayBuffer, FirstOrderActuator, HighFidelityModel,
    SensorChannel, ServoDynamics, defaults, validate,
)

DT = 0.0025
XML = Path(__file__).resolve().parents[1] / 'xml' / 'Scene.xml'


def config(**overrides):
    result = defaults()
    result.update(enable_high_fidelity=True)
    result.update(overrides)
    return result


def test_motor_exact_response_and_asymmetric_decay():
    motor = FirstOrderActuator(0.022, 0.025, np.zeros(4))
    for _ in range(40):
        actual = motor.update(np.full(4, 10.0), DT)
    np.testing.assert_allclose(actual, 10 * (1 - np.exp(-0.1 / 0.022)))
    start = actual.copy()
    for _ in range(40):
        actual = motor.update(np.zeros(4), DT)
    np.testing.assert_allclose(actual, start * np.exp(-0.1 / 0.025))


def test_servo_reference_lag_rate_and_travel():
    servo = ServoDynamics(0.04, 2.0, -1.0, 1.0, np.zeros(4))
    previous = servo.value.copy()
    for _ in range(1000):
        angle = servo.update(np.full(4, 2.0), DT)
        assert np.all(np.abs(angle - previous) <= 2 * DT + 1e-14)
        assert np.all(np.abs(angle) <= 1.0)
        previous = angle
    np.testing.assert_allclose(angle, 1.0)


def test_delay_causality_rounding_and_substep():
    delay = DelayBuffer(0.006, DT, [0.0])
    assert delay.delay == 3 * DT
    assert [delay.update(i * DT, [1.0])[0] for i in range(5)] == [0, 0, 0, 1, 1]
    tiny = DelayBuffer(0.00015, DT, [0.0])
    assert tiny.update(0.0, [1.0])[0] == 1


def test_sensor_latency_sample_hold_bias_and_no_truth_mutation():
    c = config(sensor_position_update_rate_hz=100.0, sensor_position_latency_sec=0.005,
               sensor_position_noise_std=0.0, sensor_position_initial_bias_std=0.0,
               sensor_position_bias_random_walk_std=0.0, sensor_position_bias=[1.0, 2.0, 3.0])
    sensor = SensorChannel(c, 'position', np.random.default_rng(1), DT)
    outputs = []
    for i in range(9):
        truth = np.full(3, i, dtype=float)
        outputs.append(sensor.update(i * DT, truth))
        np.testing.assert_array_equal(truth, np.full(3, i))
    np.testing.assert_array_equal(outputs[0], np.zeros(3))
    np.testing.assert_array_equal(outputs[1], np.zeros(3))
    for i in range(2, 6):
        np.testing.assert_array_equal(outputs[i], [1, 2, 3])
    np.testing.assert_array_equal(outputs[6], [5, 6, 7])


def test_all_off_matches_old_effective_delay_and_thrust():
    a = HighFidelityModel(config(enable_high_fidelity=False), 1, DT)
    c = config()
    for key in c:
        if key.startswith('enable_') and key != 'enable_high_fidelity':
            c[key] = False
    b = HighFidelityModel(c, 1, DT)
    ring = np.zeros((4, 8))
    index = 0
    for step in range(30):
        command = np.r_[np.full(4, step / 2), np.full(4, step / 100)]
        ring[index] = command
        index = (index + 1) % 4
        delayed = ring[index]
        expected = (np.clip(0.02 * delayed[:4] ** 2, 0, 15), delayed[4:])
        for model in (a, b):
            output = model.actuate(step * DT, command, [15] * 4)
            for actual, target in zip(output, expected):
                np.testing.assert_allclose(actual, target)


def test_on_motor_response_and_saturation_and_invalid_inputs():
    hf = HighFidelityModel(config(motor_command_delay=0.0, enable_motor_mismatch=False), 1, DT)
    thrust, _ = hf.actuate(0, [20] * 4 + [0] * 4, [15] * 4)
    assert np.all((thrust > 0) & (thrust < 8))
    for step in range(1, 1000):
        thrust, angle = hf.actuate(step * DT, [1e308, np.nan, np.inf, -1] + [np.nan] * 4, [15] * 4)
        assert np.isfinite(thrust).all() and np.isfinite(angle).all()
        assert np.all((thrust >= 0) & (thrust <= 15))
    np.testing.assert_allclose(thrust, [15, 0, 0, 0], atol=1e-12)


def make_physics(hf):
    model = mujoco.MjModel.from_xml_path(str(XML))
    model.opt.timestep = DT
    data = mujoco.MjData(model)
    base = model.body('drone_base').id
    ids = [model.geom('hand_palm_col').id, model.geom('contact_plate_px').id]
    hf.configure_physics(model, data, base, ids)
    return model, data, base, ids


def test_seed_repeatability_and_feature_stream_independence():
    a = HighFidelityModel(config(), 42, DT)
    b = HighFidelityModel(config(), 42, DT)
    other = HighFidelityModel(config(enable_sensor_model=False), 42, DT)
    for hf in (a, b, other):
        make_physics(hf)
    for name in ('motor_gains', 'inertia_scale', 'com_offset', 'contact_scales'):
        np.testing.assert_array_equal(getattr(a, name), getattr(b, name))
        np.testing.assert_array_equal(getattr(a, name), getattr(other, name))
    for step in range(100):
        np.testing.assert_array_equal(a.external_residual(), b.external_residual())
        np.testing.assert_array_equal(b.residual.value, other.external_residual())
        for name in a.sensors:
            x = np.ones_like(a.sensors[name].bias)
            np.testing.assert_array_equal(a.sensors[name].update(step * DT, x), b.sensors[name].update(step * DT, x))
    different = HighFidelityModel(config(), 43, DT)
    assert not np.array_equal(a.motor_gains, different.motor_gains)


def test_inertia_principal_axes_and_com_convention():
    hf = HighFidelityModel(config(inertia_random_range=0.0, com_random_range=0.0,
                                  inertia_scale=[1.02, 1.03, 1.04], com_offset=[0.001, -0.002, 0.003]), 1, DT)
    nominal = mujoco.MjModel.from_xml_path(str(XML))
    base = nominal.body('drone_base').id
    model, data, _, _ = make_physics(hf)
    np.testing.assert_allclose(model.body_inertia[base], nominal.body_inertia[base] * [1.02, 1.03, 1.04])
    np.testing.assert_allclose(model.body_ipos[base], nominal.body_ipos[base] + [0.001, -0.002, 0.003])
    np.testing.assert_array_equal(model.body_iquat, nominal.body_iquat)
    rotation = data.xmat[base].reshape(3, 3)
    np.testing.assert_allclose(data.xipos[base] - data.xpos[base], rotation @ model.body_ipos[base], atol=1e-15)
    assert np.linalg.norm(data.xipos[base] - data.xpos[base]) > 0
    np.testing.assert_array_equal(model.body_mass, nominal.body_mass)
    for _ in range(400):
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()
    assert not np.any(data.warning.number)


@pytest.mark.parametrize('ref', [[0.02, 1.0], [-10000.0, -100.0]])
def test_contact_scaling_preserves_case_configuration(ref):
    c = config(contact_random_range=0.0, contact_friction_scale=1.2,
               contact_time_constant_scale=2.0, contact_damping_scale=0.5)
    hf = HighFidelityModel(c, 1, DT)
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    ids = [model.geom('hand_palm_col').id, model.geom('contact_plate_px').id]
    model.geom_solref[ids] = ref
    before = model.geom_friction.copy()
    condim, masks, solimp = model.geom_condim.copy(), model.geom_contype.copy(), model.geom_solimp.copy()
    hf.configure_physics(model, data, model.body('drone_base').id, ids)
    np.testing.assert_allclose(model.geom_friction[ids], before[ids] * 1.2)
    expected = [0.04, 0.5] if ref[0] > 0 else [-2500, -50]
    np.testing.assert_allclose(model.geom_solref[ids], [expected] * 2)
    np.testing.assert_array_equal(model.geom_condim, condim)
    np.testing.assert_array_equal(model.geom_contype, masks)
    np.testing.assert_array_equal(model.geom_solimp, solimp)


@pytest.mark.parametrize('key,value', [
    ('motor_time_constant_up', -0.1), ('servo_rate_limit', 0),
    ('servo_command_delay', -1), ('motor_gains', [1, 1]),
    ('inertia_scale', [1, 0, 1]), ('sensor_gyro_noise_std', -1),
    ('sensor_position_update_rate_hz', 401), ('com_offset', [0, np.nan, 0]),
    ('contact_random_range', 1), ('residual_time_constant', 0),
])
def test_validation(key, value):
    with pytest.raises(ValueError):
        validate(config(**{key: value}), DT)


def test_invalid_inertia_triangle_rejected():
    hf = HighFidelityModel(config(inertia_scale=[1.0, 1.0, 100.0], inertia_random_range=0.0), 1, DT)
    with pytest.raises(ValueError, match='triangle'):
        make_physics(hf)


def test_ou_stationary_std_and_temporal_correlation():
    process = ColoredWrenchProcess(0.2, np.full(6, 0.05), np.random.default_rng(1))
    samples = np.array([process.update(DT) for _ in range(40000)])[1000:]
    np.testing.assert_allclose(samples.std(axis=0), 0.05, rtol=0.15)
    assert np.corrcoef(samples[:-1, 0], samples[1:, 0])[0, 1] > 0.97
