"""ROS integration checks for the four layer wrench controller."""
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
from std_msgs.msg import Bool, Float64MultiArray
from rclpy.parameter import Parameter
from rcl_interfaces.srv import ListParameters, SetParameters


@pytest.mark.parametrize('custom', [False, True])
def test_four_layer_cascade(custom, tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', str(120 + os.getpid() % 70 + int(custom)))
    rclpy.init()
    node = rclpy.create_node('cascade_regression')
    data = {key: [] for key in ('wrench', 'att_ref', 'position', 'velocity', 'attitude', 'rate')}
    node.create_subscription(Wrench, '/minitrone/wrench_cmd', data['wrench'].append, 10)
    node.create_subscription(AttitudeCmd, '/minitrone/att_ref', data['att_ref'].append, 10)
    for key in ('position', 'velocity', 'attitude', 'rate'):
        node.create_subscription(Float64MultiArray, '/minitrone/controller_debug/' + key,
                                 data[key].append, 10)
    pubs = {
        'state': node.create_publisher(MinitroneState, '/minitrone/state', 10),
        'cmd': node.create_publisher(Cmd, '/minitrone/cmd', 10),
        'att': node.create_publisher(AttitudeCmd, '/minitrone/att_cmd', 10),
        'adm': node.create_publisher(Cmd, '/minitrone/cmd_admittance', 10),
        'adm_att': node.create_publisher(AttitudeCmd, '/minitrone/att_cmd_admittance', 10),
        'active': node.create_publisher(Bool, '/minitrone/admittance_active', 10),
        'reset': node.create_publisher(Bool, '/minitrone/controller_reset', 10),
        'allocator_saturated': node.create_publisher(Bool, '/minitrone/allocator_saturated', 10),
        'clock': node.create_publisher(Clock, '/clock', 10),
    }
    exe = Path(get_package_prefix('minitrone_controller')) / 'lib/minitrone_controller/minitrone_wrench_controller'
    args = [str(exe), '--ros-args', '-p', 'use_sim_time:=true']
    if custom:
        # Isolate one channel in each layer to verify the cascade and output limit.
        nonzero = {'KP_POS_X': 2.0, 'KP_VEL_X': 3.0,
                   'KP_ATT_YAW': 2.0, 'KP_RATE_YAW': 3.0}
        for layer, axes in (('POS', ('X', 'Y', 'Z')), ('VEL', ('X', 'Y', 'Z')),
                            ('ATT', ('ROLL', 'PITCH', 'YAW')),
                            ('RATE', ('ROLL', 'PITCH', 'YAW'))):
            for axis in axes:
                for term in ('KP', 'KI', 'KD'):
                    name = f'{term}_{layer}_{axis}'
                    args += ['-p', f'{name}:={nonzero.get(name, 0.0)}']
        args += ['-p', 'velocity_limit_x:=0.4']
    log = (tmp_path / 'controller.log').open('w')
    process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)

    def spin_for(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.005)

    def send(key, msg):
        pubs[key].publish(msg)
        spin_for(.03)

    def sample(sec, pos=(0., 0., 0.), vel=(0., 0., 0.), rpy=(0., 0., 0.)):
        clock = Clock()
        clock.clock.sec = int(sec)
        clock.clock.nanosec = round((sec-int(sec))*1e9)
        send('clock', clock)
        state = MinitroneState()
        state.pos, state.vel, state.rpy = list(pos), list(vel), list(rpy)
        state.acc = [float('nan')] * 3  # This field is deliberately ignored.
        count = len(data['wrench'])
        pubs['state'].publish(state)
        deadline = time.monotonic() + 2.
        while len(data['wrench']) == count and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        assert len(data['wrench']) == count + 1
        spin_for(.03)
        return data['wrench'][-1]

    try:
        deadline = time.monotonic() + 8.
        while not all(pub.get_subscription_count() for pub in pubs.values()):
            assert process.poll() is None
            assert time.monotonic() < deadline, 'ROS discovery timeout'
            spin_for(.05)

        list_client = node.create_client(ListParameters, '/minitrone_wrench_controller/list_parameters')
        set_client = node.create_client(SetParameters, '/minitrone_wrench_controller/set_parameters')
        assert list_client.wait_for_service(timeout_sec=3.)
        assert set_client.wait_for_service(timeout_sec=3.)
        future = list_client.call_async(ListParameters.Request(depth=10))
        while not future.done():
            rclpy.spin_once(node, timeout_sec=.01)
        gain_names = {name for name in future.result().result.names
                      if name.startswith(('KP_', 'KI_', 'KD_'))}
        expected_names = {f'{term}_{layer}_{axis}'
                          for layer, axes in (('POS', ('X', 'Y', 'Z')),
                                              ('VEL', ('X', 'Y', 'Z')),
                                              ('ATT', ('ROLL', 'PITCH', 'YAW')),
                                              ('RATE', ('ROLL', 'PITCH', 'YAW')))
                          for axis in axes for term in ('KP', 'KI', 'KD')}
        assert gain_names == expected_names
        assert len(gain_names) == 36

        if custom:
            for _ in range(10):
                send('cmd', Cmd(pos_cmd=[1., 0., 0.]))
                send('att', AttitudeCmd(yaw_ref=10.))
            output = sample(1.)
            np.testing.assert_allclose(output.force, [2.86 * 3. * .4, 0., 2.86 * 9.81], atol=2e-5)
            np.testing.assert_allclose(data['position'][-1].data[18:21], [.4, 0., 0.], atol=1e-6)
            np.testing.assert_allclose(data['att_ref'][-1].yaw_ref, 10., atol=1e-6)
            assert output.moment[2] > 0.
            old_torque = output.moment[2]
            request = SetParameters.Request()
            request.parameters = [Parameter('KP_RATE_YAW', value=4.0).to_parameter_msg()]
            change = set_client.call_async(request)
            while not change.done():
                rclpy.spin_once(node, timeout_sec=.01)
            assert change.result().results[0].successful
            output = sample(1.05)
            assert output.moment[2] > old_torque
            request.parameters = [Parameter('KI_POS_X', value=1.0).to_parameter_msg()]
            change = set_client.call_async(request)
            while not change.done():
                rclpy.spin_once(node, timeout_sec=.01)
            assert change.result().results[0].successful
            send('cmd', Cmd(pos_cmd=[.1, 0., 0.]))
            send('allocator_saturated', Bool(data=True))
            sample(1.06)
            np.testing.assert_allclose(data['position'][-1].data[12], 0., atol=1e-9)
            send('allocator_saturated', Bool(data=False))
            sample(1.07)
            assert data['position'][-1].data[12] > 0.
            send('adm', Cmd(pos_cmd=[1., 0., 0.]))
            send('adm_att', AttitudeCmd(yaw_ref=5.))
            send('active', Bool(data=True))
            sample(1.1)
            np.testing.assert_allclose(data['att_ref'][-1].yaw_ref, 15., atol=1e-6)
            send('reset', Bool(data=True))
            sample(1.2)
            for key in ('position', 'velocity', 'attitude', 'rate'):
                assert np.isfinite(data[key][-1].data).all()
        else:
            # Stationary hover reference at the initial pose and initial yaw.
            yaw = .7
            for _ in range(10):
                send('cmd', Cmd(pos_cmd=[0., 0., 0.]))
                send('att', AttitudeCmd(yaw_ref=yaw*180./math.pi))
            for tick in range(1, 8):
                output = sample(tick*.01, rpy=[0., 0., yaw])
            np.testing.assert_allclose(output.force, [0., 0., 2.86*9.81], atol=1e-3)
            np.testing.assert_allclose(output.moment, [0., 0., 0.], atol=1e-3)
            for key in ('position', 'velocity', 'attitude', 'rate'):
                assert np.isfinite(data[key][-1].data).all()
                np.testing.assert_allclose(data[key][-1].data[6:9], 0., atol=1e-6)
                np.testing.assert_allclose(data[key][-1].data[12:18], 0., atol=1e-6)
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
