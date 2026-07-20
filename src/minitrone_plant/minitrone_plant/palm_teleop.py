#!/usr/bin/env python3
import math
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0


class PalmTeleop(Node):
    def __init__(self):
        super().__init__('minitrone_palm_teleop')
        self.speed_xyz = float(self.declare_parameter('speed_xyz', 0.05).value)
        self.speed_rpy = float(self.declare_parameter('speed_rpy_deg', 1.0).value) * DEG2RAD
        self.command_timeout = float(self.declare_parameter('command_timeout', 0.05).value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 100.0).value)
        self.pub_pose = self.create_publisher(Float64MultiArray, '/minitrone/palm_pose_cmd', 10)
        self.sub_pose = self.create_subscription(Float64MultiArray, '/minitrone/palm_pose_state', self._palm_pose_state_callback, 10)
        self.pose = None
        self.reset_pose = None
        self.v = [0.0] * 6
        self.last_key_time = self.get_clock().now()
        self.last_update_time = self.last_key_time
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1.0), self._on_timer)
        self._stdin_fd = sys.stdin.fileno()
        self._stdin_old = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        self._print_help()

    def destroy_node(self):
        termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._stdin_old)
        return super().destroy_node()

    def _print_help(self):
        self.get_logger().info(
            'Velocity-integrated palm teleop keys: W/S +x/-x, A/D +y/-y, '
            'Q/E +z/-z, R/F yaw +/- , T/G pitch +/- , Y/H roll +/- , '
            'Space stop, 0 reset, X quit.'
        )

    def _palm_pose_state_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 6 or self.pose is not None:
            return
        self.pose = list(msg.data[:6])
        self.reset_pose = self.pose.copy()
        self.get_logger().info(
            f"teleop initialized from current palm pose: pos={[round(v, 3) for v in self.pose[:3]]} "
            f"rpy_deg={[round(v * RAD2DEG, 1) for v in self.pose[3:6]]}"
        )

    def _publish_pose(self):
        if self.pose is None:
            return
        msg = Float64MultiArray()
        msg.data = self.pose.copy()
        self.pub_pose.publish(msg)

    def _print_pose(self):
        self.get_logger().info(
            f"palm pose: pos={[round(v, 3) for v in self.pose[:3]]} "
            f"rpy_deg={[round(v * RAD2DEG, 1) for v in self.pose[3:6]]} "
            f"vel={[round(v, 3) for v in self.v[:3]]} "
            f"rpy_vel_deg={[round(v * RAD2DEG, 1) for v in self.v[3:6]]}"
        )

    def _read_key(self):
        if not select.select([sys.stdin], [], [], 0.0)[0]:
            return None
        return sys.stdin.read(1)

    def _apply_key(self, key):
        if key is None:
            return
        if key.lower() == 'x':
            raise KeyboardInterrupt

        if self.pose is None:
            return

        changed = True
        if key.lower() == 'w':
            self.v = [self.speed_xyz, 0.0, 0.0, 0.0, 0.0, 0.0]
        elif key.lower() == 's':
            self.v = [-self.speed_xyz, 0.0, 0.0, 0.0, 0.0, 0.0]
        elif key.lower() == 'a':
            self.v = [0.0, self.speed_xyz, 0.0, 0.0, 0.0, 0.0]
        elif key.lower() == 'd':
            self.v = [0.0, -self.speed_xyz, 0.0, 0.0, 0.0, 0.0]
        elif key.lower() == 'q':
            self.v = [0.0, 0.0, self.speed_xyz, 0.0, 0.0, 0.0]
        elif key.lower() == 'e':
            self.v = [0.0, 0.0, -self.speed_xyz, 0.0, 0.0, 0.0]
        elif key.lower() == 'r':
            self.v = [0.0, 0.0, 0.0, 0.0, 0.0, self.speed_rpy]
        elif key.lower() == 'f':
            self.v = [0.0, 0.0, 0.0, 0.0, 0.0, -self.speed_rpy]
        elif key.lower() == 't':
            self.v = [0.0, 0.0, 0.0, 0.0, self.speed_rpy, 0.0]
        elif key.lower() == 'g':
            self.v = [0.0, 0.0, 0.0, 0.0, -self.speed_rpy, 0.0]
        elif key.lower() == 'y':
            self.v = [0.0, 0.0, 0.0, self.speed_rpy, 0.0, 0.0]
        elif key.lower() == 'h':
            self.v = [0.0, 0.0, 0.0, -self.speed_rpy, 0.0, 0.0]
        elif key == ' ':
            self.v = [0.0] * 6
        elif key == '0':
            self.pose[:] = self.reset_pose.copy()
            self.v = [0.0] * 6
            self._publish_pose()
        else:
            changed = False

        if changed:
            self.last_key_time = self.get_clock().now()
            self._print_pose()

    def _integrate_pose(self):
        if self.pose is None:
            return
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        if dt <= 0.0 or dt > 0.2:
            return

        age = (now - self.last_key_time).nanoseconds * 1e-9
        if age > self.command_timeout:
            self.v = [0.0] * 6

        for i in range(6):
            self.pose[i] += self.v[i] * dt

    def _on_timer(self):
        self._apply_key(self._read_key())
        self._integrate_pose()
        self._publish_pose()


def main():
    rclpy.init()
    node = PalmTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
