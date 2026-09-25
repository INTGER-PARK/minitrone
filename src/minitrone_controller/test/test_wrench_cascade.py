"""Installed ROS integration regression: run with sourced install/setup.bash."""
import math
import os
from pathlib import Path
import subprocess
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from ament_index_python.packages import get_package_prefix
from minitrone_interfaces.msg import AttitudeCmd, Cmd, MinitroneState, Wrench
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool


def rotation(rpy):
    r, p, y = rpy
    sr, cr, sp, cp, sy, cy = math.sin(r), math.cos(r), math.sin(p), math.cos(p), math.sin(y), math.cos(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


@pytest.mark.parametrize('mass', [2.5, 4.0])
@pytest.mark.parametrize('custom', [False, True])
def test_cascade_installed(custom, mass, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(100 + os.getpid() % 80 + 2*int(custom) + int(mass == 4.0)))
    rclpy.init()
    node = rclpy.create_node('cascade_regression')
    outputs = []
    attitude_refs = []
    node.create_subscription(Wrench, '/minitrone/wrench_cmd', outputs.append, 10)
    node.create_subscription(AttitudeCmd, '/minitrone/att_ref', attitude_refs.append, 10)
    publishers = {
        'state': node.create_publisher(MinitroneState, '/minitrone/state', 10),
        'cmd': node.create_publisher(Cmd, '/minitrone/cmd', 10),
        'adm': node.create_publisher(Cmd, '/minitrone/cmd_admittance', 10),
        'att': node.create_publisher(AttitudeCmd, '/minitrone/att_cmd_admittance', 10),
        'base_att': node.create_publisher(AttitudeCmd, '/minitrone/att_cmd', 10),
        'active': node.create_publisher(Bool, '/minitrone/admittance_active', 10),
        'clock': node.create_publisher(Clock, '/clock', 10),
    }
    executable = Path(get_package_prefix('minitrone_controller')) / 'lib/minitrone_controller/minitrone_wrench_controller'
    args = [str(executable), '--ros-args', '-p', 'use_sim_time:=true', '-p', f'mass:={mass}']
    if custom:
        # Exercise independent gains, outer speed limiting, and inner I/D.
        for key, value in {'position_kp_x': 2., 'position_ki_x': 0.,
                           'velocity_kp_x': 3., 'velocity_ki_x': 1.,
                           'velocity_kd_x': .5, 'velocity_limit_x': .4}.items():
            args += ['-p', f'{key}:={value}']
    log = (tmp_path / 'controller.log').open('w')
    process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)

    def spin_for(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.005)

    def send(key, msg):
        publishers[key].publish(msg)
        spin_for(.04)

    def sample(sec, vel, pos=(0., 0., 0.), rpy=(0., 0., 0.), nan_acc=False):
        clock = Clock()
        clock.clock.sec = int(sec)
        clock.clock.nanosec = round((sec-int(sec))*1e9)
        send('clock', clock)
        state = MinitroneState()
        state.pos, state.vel, state.rpy = list(pos), list(vel), list(rpy)
        if nan_acc:
            state.acc = [float('nan')] * 3  # Must not use this frame-ambiguous field.
        count = len(outputs)
        publishers['state'].publish(state)
        until = time.monotonic() + 2.
        while len(outputs) == count and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.01)
        assert len(outputs) == count + 1
        return np.array(outputs[-1].force), np.array(outputs[-1].moment)

    try:
        deadline = time.monotonic() + 8.
        while not all(p.get_subscription_count() for p in publishers.values()):
            assert process.poll() is None
            assert time.monotonic() < deadline, 'ROS discovery timeout'
            spin_for(.05)
        # Graph discovery can precede DDS writer/reader matching. Repeat the
        # initial reference before sending any state (no integrator updates).
        for _ in range(10):
            send('cmd', Cmd(pos_cmd=[1., -.5, .3]))
        if custom:
            force, _ = sample(1, [0., 0., 0.])
            np.testing.assert_allclose(force[0], mass*(3*.4 + .4*.0025), atol=1e-5)
            force, _ = sample(1.1, [.1, 0., 0.], nan_acc=True)
            np.testing.assert_allclose(force[0], mass*(3*.3 + .001 + .3*.1 - .5), atol=1e-5)
            send('active', Bool(data=True))  # Reset both loops even before adm refs arrive.
            force, _ = sample(1.2, [.1, 0., 0.])
            np.testing.assert_allclose(force[0], mass*(3*.3 + .3*.1), atol=1e-5)
            return

        integral = np.zeros(3)
        kp, ki, kd = np.array([28., 28., 24.]), np.array([1.5, 1.5, 1.2]), np.array([6., 6., 10.])
        # Large clock gaps select the documented 400 Hz fallback deterministically.
        for sec, pos, vel, rpy in [
            (1, [0., 0., 0.], [0., 0., 0.], [0., 0., 0.]),
            (2, [.1, -.2, .1], [.3, -.4, .2], [0., 0., math.pi/2]),
            (3, [-20., 20., -20.], [2., -3., 1.], [.4, -.3, .8]),
            (4, [10000., -10000., 10000.], [0., 0., 0.], [-.3, .5, -1.]),
        ]:
            error = np.array([1., -.5, .3]) - pos
            integral = np.clip(integral + ki*error*.0025, -5., 100.)
            expected_world = np.clip(kp*error + integral - kd*vel, -200., 200.) + [0., 0., mass*9.81]
            force, moment = sample(sec, vel, pos, rpy)
            np.testing.assert_allclose(force, rotation(rpy).T @ expected_world, atol=2e-5)
            np.testing.assert_allclose(moment, np.clip(-6*np.array(rpy), -5., 5.), atol=1e-6)

        # The implicit level attitude reference is visible as an AttitudeCmd.
        assert attitude_refs
        np.testing.assert_allclose(
            [attitude_refs[-1].roll_ref, attitude_refs[-1].pitch_ref,
             attitude_refs[-1].yaw_ref], [0., 0., 0.], atol=1e-6)

        send('base_att', AttitudeCmd(roll_ref=10., pitch_ref=-5., yaw_ref=15.))
        sample(4.5, [0., 0., 0.])
        spin_for(.05)
        np.testing.assert_allclose(
            [attitude_refs[-1].roll_ref, attitude_refs[-1].pitch_ref,
             attitude_refs[-1].yaw_ref], [10., -5., 15.], atol=1e-5)

        # Admittance selects its references and clears the saturated integrals.
        send('adm', Cmd(pos_cmd=[.2, .1, .4]))
        send('att', AttitudeCmd(roll_ref=2., pitch_ref=-3., yaw_ref=4.))
        send('active', Bool(data=True))
        force, moment = sample(5, [0., 0., 0.], [.2, .1, .4])
        spin_for(.05)
        np.testing.assert_allclose(force, [0., 0., mass*9.81], atol=2e-6)
        np.testing.assert_allclose(
            [attitude_refs[-1].roll_ref, attitude_refs[-1].pitch_ref,
             attitude_refs[-1].yaw_ref], [12., -8., 19.], atol=1e-5)
        # OFF captures measured pose; stale teleop references must be ignored.
        send('active', Bool(data=False))
        send('cmd', Cmd(pos_cmd=[9., 9., 9.]))
        force, _ = sample(6, [0., 0., 0.], [.2, .1, .4])
        np.testing.assert_allclose(force, [0., 0., mass*9.81], atol=2e-6)
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
