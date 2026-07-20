#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from minitrone_interfaces.msg import Cmd, MinitroneState


class PositionTeleop(Node):
    def __init__(self):
        super().__init__("minitrone_position_teleop")

        self.x = float(self.declare_parameter("x0", 0.0).value)
        self.y = float(self.declare_parameter("y0", 0.0).value)
        self.z = float(self.declare_parameter("z0", 1.0).value)
        self.reset = [self.x, self.y, self.z]
        self.speed_xy = float(self.declare_parameter("speed_xy", 0.3).value)
        self.speed_z = float(self.declare_parameter("speed_z", 0.2).value)
        self.command_timeout = float(self.declare_parameter("command_timeout", 0.6).value)
        self.rate_hz = float(self.declare_parameter("rate_hz", 100.0).value)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.last_key_time = self.get_clock().now()
        self.last_update_time = self.last_key_time

        self.pub_cmd = self.create_publisher(Cmd, "/minitrone/cmd", 10)
        self.sub_state = self.create_subscription(
            MinitroneState, "/minitrone/state", self._on_state, 10
        )
        self.sub_admittance_active = self.create_subscription(
            Bool, "/minitrone/admittance_active", self._on_admittance_active, 10
        )
        self.latest_position = None
        self.admittance_active = None
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1.0), self._on_timer)

        self._stdin_fd = sys.stdin.fileno()
        self._stdin_old = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)

        self._print_help()
        self._print_cmd()

    def destroy_node(self):
        termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._stdin_old)
        return super().destroy_node()

    def _print_help(self):
        self.get_logger().info(
            "Velocity-integrated position teleop keys: W/S +x/-x, A/D +y/-y, "
            "R/F +z/-z, arrows move x/y, [/] z up/down, Space stop, 0 reset, "
            "Q quit."
        )

    def _print_cmd(self):
        self.get_logger().info(
            f"position cmd: x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}, "
            f"vel: vx={self.vx:.3f}, vy={self.vy:.3f}, vz={self.vz:.3f}"
        )

    def _publish_cmd(self):
        msg = Cmd()
        msg.pos_cmd[0] = float(self.x)
        msg.pos_cmd[1] = float(self.y)
        msg.pos_cmd[2] = float(self.z)
        self.pub_cmd.publish(msg)

    def _on_state(self, msg):
        self.latest_position = [float(value) for value in msg.pos]

    def _on_admittance_active(self, msg):
        was_active = self.admittance_active is True
        self.admittance_active = bool(msg.data)
        if not was_active or self.admittance_active or self.latest_position is None:
            return

        self.x, self.y, self.z = self.latest_position
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.last_update_time = self.get_clock().now()
        self.get_logger().info(
            "admittance OFF: position command synchronized to current pose"
        )
        self._print_cmd()

    def _read_key(self):
        if not select.select([sys.stdin], [], [], 0.0)[0]:
            return None

        key = sys.stdin.read(1)
        if key == "\x1b":
            seq = sys.stdin.read(2)
            return key + seq
        return key

    def _apply_key(self, key):
        changed = True

        if key is None:
            return
        if key.lower() == "q":
            raise KeyboardInterrupt

        if key == "\x1b[A":
            self.vx = self.speed_xy
            self.vy = 0.0
            self.vz = 0.0
        elif key == "\x1b[B":
            self.vx = -self.speed_xy
            self.vy = 0.0
            self.vz = 0.0
        elif key == "\x1b[D":
            self.vx = 0.0
            self.vy = self.speed_xy
            self.vz = 0.0
        elif key == "\x1b[C":
            self.vx = 0.0
            self.vy = -self.speed_xy
            self.vz = 0.0
        elif key.lower() == "w":
            self.vx = self.speed_xy
            self.vy = 0.0
            self.vz = 0.0
        elif key.lower() == "s":
            self.vx = -self.speed_xy
            self.vy = 0.0
            self.vz = 0.0
        elif key.lower() == "a":
            self.vx = 0.0
            self.vy = self.speed_xy
            self.vz = 0.0
        elif key.lower() == "d":
            self.vx = 0.0
            self.vy = -self.speed_xy
            self.vz = 0.0
        elif key.lower() == "r" or key == "[":
            self.vx = 0.0
            self.vy = 0.0
            self.vz = self.speed_z
        elif key.lower() == "f" or key == "]":
            self.vx = 0.0
            self.vy = 0.0
            self.vz = -self.speed_z
        elif key == " ":
            self.vx = 0.0
            self.vy = 0.0
            self.vz = 0.0
        elif key == "0":
            self.x, self.y, self.z = self.reset
            self.vx = 0.0
            self.vy = 0.0
            self.vz = 0.0
        else:
            changed = False

        if changed:
            self.last_key_time = self.get_clock().now()
            self._print_cmd()

    def _integrate_position_cmd(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        if dt <= 0.0 or dt > 0.2:
            return

        age = (now - self.last_key_time).nanoseconds * 1e-9
        if age > self.command_timeout:
            self.vx = 0.0
            self.vy = 0.0
            self.vz = 0.0

        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

    def _on_timer(self):
        self._apply_key(self._read_key())
        self._integrate_position_cmd()
        self._publish_cmd()


def main():
    rclpy.init()
    node = PositionTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
