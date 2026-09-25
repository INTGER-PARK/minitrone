"""Installed ros2-run smoke test with actual topic reception (no controller)."""
import os
import signal
import subprocess
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from ament_index_python.packages import get_package_share_directory
from minitrone_interfaces.msg import Input, MinitroneState, MobObserverInput


@pytest.mark.parametrize('enabled', [False, True])
def test_installed_headless_plant(enabled, tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    log_path = tmp_path / f'plant_{enabled}.log'
    samples, states = [], []
    rclpy.init()
    listener = rclpy.create_node('hf_smoke_listener')
    listener.create_subscription(MobObserverInput, '/minitrone/mob_observer_input', samples.append, 100)
    listener.create_subscription(MinitroneState, '/minitrone/state', states.append, 100)
    publisher = listener.create_publisher(Input, '/minitrone/input', 10)
    with log_path.open('w') as log:
        process = subprocess.Popen([
            'ros2', 'run', 'minitrone_plant', 'minitrone_plant', '--ros-args',
            '--params-file', os.path.join(get_package_share_directory('minitrone_plant'), 'config', 'high_fidelity.yaml'),
            '-p', 'enable_viewer:=false', '-p', f'enable_high_fidelity:={str(enabled).lower()}',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            start = time.monotonic()
            while time.monotonic() - start < 4.0:
                elapsed = time.monotonic() - start
                command = Input()
                command.u = [15.0 if elapsed > 1.0 else 0.0] * 4 + [0.0] * 4
                publisher.publish(command)
                rclpy.spin_once(listener, timeout_sec=0.005)
                assert process.poll() is None, log_path.read_text()
            assert len(samples) > 800 and len(states) > 800
            for msg in samples:
                assert np.isfinite([*msg.pos, *msg.vel, *msg.rpy, *msg.w_rpy,
                                    *msg.actuation_force, *msg.actuation_moment]).all()
            for msg in states:
                assert np.isfinite([*msg.pos, *msg.vel, *msg.acc, *msg.rpy,
                                    *msg.w_rpy, *msg.a_rpy, *msg.servo]).all()
            steps = np.array([s.step for s in samples])
            times = np.array([s.sim_time for s in samples])
            assert np.all(np.diff(steps) > 0)
            np.testing.assert_allclose(np.diff(times), np.diff(steps) * 0.0025, atol=1e-10)
            forces = np.array([s.actuation_force[2] for s in samples])
            assert forces.max() > 10.0
            if enabled:
                assert np.count_nonzero((forces > 0.1) & (forces < 10.0)) >= 5
            else:
                assert np.count_nonzero((forces > 0.1) & (forces < 10.0)) <= 1
            print(f'HF={enabled}: {len(samples)} observer / {len(states)} state packets; '
                  f'sim duration={times[-1] - times[0]:.3f}s; finite values; 400 Hz step timestamps')
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            listener.destroy_node()
            rclpy.shutdown()
    contents = log_path.read_text()
    assert 'Traceback' not in contents and 'WARNING' not in contents and 'Nan' not in contents, contents
