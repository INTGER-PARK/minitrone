#!/usr/bin/env python3
"""Run a bounded, headless plant + controller + allocator hover check.

Run after sourcing /opt/ros/humble/setup.bash and install/setup.bash.
"""
import math
import os
from collections import deque
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from minitrone_interfaces.msg import AttitudeCmd, Cmd, Input, MinitroneState, Wrench
from std_msgs.msg import Bool


def executable(package, name):
    return str(Path(get_package_prefix(package)) / 'lib' / package / name)


def main():
    os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 40)
    high_fidelity = os.environ.get('HOVER_HIGH_FIDELITY', 'false').lower() == 'true'
    rclpy.init()
    node = rclpy.create_node('hover_closed_loop_check')
    states, wrenches, allocated, inputs, saturation = (deque(maxlen=3000) for _ in range(5))
    node.create_subscription(MinitroneState, '/minitrone/state', states.append, 10)
    node.create_subscription(Wrench, '/minitrone/wrench_cmd', wrenches.append, 10)
    node.create_subscription(Wrench, '/minitrone/allocated_wrench_estimate', allocated.append, 10)
    node.create_subscription(Input, '/minitrone/input', inputs.append, 10)
    node.create_subscription(Bool, '/minitrone/allocator_saturated', saturation.append, 10)
    pub_pos = node.create_publisher(Cmd, '/minitrone/cmd', 10)
    pub_att = node.create_publisher(AttitudeCmd, '/minitrone/att_cmd', 10)
    processes = []
    logs = []
    try:
        for package, name, extra in (
            ('minitrone_plant', 'minitrone_plant', ['--ros-args', '-p', 'enable_viewer:=false',
                                                   '-p', f'enable_high_fidelity:={str(high_fidelity).lower()}',
                                                   '-p', 'initial_base_z:=1.0']),
            ('minitrone_controller', 'minitrone_wrench_controller', []),
            ('minitrone_controller', 'minitrone_allocator_controller', []),
        ):
            log = Path('/tmp') / (name + '_hover_check.log')
            logs.append(log.open('w'))
            processes.append(subprocess.Popen([executable(package, name), *extra],
                                              stdout=logs[-1], stderr=subprocess.STDOUT))
        deadline = time.monotonic() + 8.0
        while not states and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert states, 'no plant state received'
        initial = states[-1]
        pos_sp = np.asarray(initial.pos)
        yaw_sp = float(initial.rpy[2])
        pos_msg = Cmd()
        pos_msg.pos_cmd = pos_sp.tolist()
        att_msg = AttitudeCmd(roll_ref=0., pitch_ref=0., yaw_ref=yaw_sp*180./math.pi)
        start = time.monotonic()
        while time.monotonic() - start < 8.0:
            pub_pos.publish(pos_msg)
            pub_att.publish(att_msg)
            rclpy.spin_once(node, timeout_sec=.01)
            assert all(p.poll() is None for p in processes), 'a node exited during hover'
        assert len(states) > 100 and wrenches and allocated and inputs and saturation
        trajectory = np.asarray([s.pos for s in states])
        velocity = np.asarray([s.vel for s in states])
        attitude = np.asarray([s.rpy for s in states])
        rates = np.asarray([s.w_rpy for s in states])
        force = np.asarray([w.force for w in wrenches])
        torque = np.asarray([w.moment for w in wrenches])
        allocated_force = np.asarray([w.force for w in allocated])
        allocated_torque = np.asarray([w.moment for w in allocated])
        actuator = np.asarray([m.u for m in inputs])
        assert all(np.isfinite(x).all() for x in (trajectory, velocity, attitude,
                                                   rates, force, torque, allocated_force,
                                                   allocated_torque, actuator))
        position_error = pos_sp - trajectory[-1]
        attitude_error = np.array([0., 0., yaw_sp]) - attitude[-1]
        saturation_fraction = np.mean([s.data for s in saturation])
        max_excursion = np.max(np.linalg.norm(trajectory - pos_sp, axis=1))
        # Bounded hover acceptance after an 8 s settle, with both nominal and
        # randomized high-fidelity plant models. Startup transients are allowed.
        assert np.linalg.norm(position_error) < .05
        assert np.linalg.norm(velocity[-1]) < .05
        assert np.linalg.norm(attitude_error) < .05
        assert np.linalg.norm(rates[-1]) < .05
        assert max_excursion < .5
        assert saturation_fraction < .05
        assert actuator[:, :4].min() >= 0. and actuator[:, :4].max() <= math.sqrt(15./.02) + 1e-6
        assert np.max(np.abs(actuator[:, 4:])) <= math.radians(65.) + 1e-6
        print('initial_position', pos_sp.tolist())
        print('high_fidelity', high_fidelity)
        print('final_position_error', position_error.tolist())
        print('final_velocity', velocity[-1].tolist())
        print('final_attitude_error', attitude_error.tolist())
        print('final_body_rate', rates[-1].tolist())
        print('final_force', force[-1].tolist())
        print('final_torque', torque[-1].tolist())
        print('final_allocated_force_estimate', allocated_force[-1].tolist())
        print('final_allocated_torque_estimate', allocated_torque[-1].tolist())
        print('motor_command_range', actuator[:, :4].min(), actuator[:, :4].max())
        print('servo_command_range', actuator[:, 4:].min(), actuator[:, 4:].max())
        print('allocator_saturation_fraction', saturation_fraction)
        print('max_position_excursion', max_excursion)
    finally:
        for process in processes:
            process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
