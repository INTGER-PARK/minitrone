#!/usr/bin/env python3
import os, time, math, threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import mujoco
import mujoco.viewer

from minitrone_interfaces.msg import Input, MinitroneState

PHYSICS_HZ = 400.0
ZETA = 0.02          # thrust = ZETA * omega^2  (minitrone allocator/plant convention)
DELAY_TIME = 0.01
RAD2DEG = 180.0 / math.pi

SIG_POS   = 1e-3
SIG_VEL   = 1e-3
SIG_GYRO  = 1e-3
SIG_SERVO = 1e-4

COM_PRINT_PERIOD_S = 1.0


def quat_to_rpy(q_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = [float(v) for v in q_wxyz]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return np.array([roll, pitch, yaw], dtype=float)

def rpy_to_R_WB(rpy: np.ndarray) -> np.ndarray:
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    sr, cr = math.sin(r), math.cos(r)
    sp, cp = math.sin(p), math.cos(p)
    sy, cy = math.sin(y), math.cos(y)

    return np.array([
        [ cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [ sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [   -sp,             cp*sr,             cp*cr]
    ], dtype=float)


class PlantRosNode(Node):
    def __init__(self):
        super().__init__("minitrone_plant")  # 이름 유지
        self.enable_viewer = bool(self.declare_parameter("enable_viewer", True).value)

        # -------- Load MuJoCo model --------
        pkg_share = get_package_share_directory("minitrone_plant")
        xml_path = os.path.join(pkg_share, "xml", "Scene.xml")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 1.0 / PHYSICS_HZ

        # --- printing ---
        self._last_com_print_t = 0.0

        def aid(name: str) -> int:
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name.encode())
            if idx < 0:
                raise RuntimeError(f"Actuator '{name}' not found in XML")
            return int(idx)

        def sid(name: str) -> int:
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name.encode())
            if idx < 0:
                raise RuntimeError(f"Sensor '{name}' not found in XML")
            return int(idx)

        def bid(name: str) -> int:
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name.encode())
            if idx < 0:
                raise RuntimeError(f"Body '{name}' not found in XML")
            return int(idx)

        # ===================== MINITRONE actuators =====================
        # XML actuator names follow Palletrone.xml:
        #   general:  BLDC1..BLDC4
        #   position: servo1_pos..servo4_pos
        self.aid_prop = [aid("BLDC1"), aid("BLDC2"), aid("BLDC3"), aid("BLDC4")]
        self.aid_servo = [aid("servo1_pos"), aid("servo2_pos"), aid("servo3_pos"), aid("servo4_pos")]

        # ===================== MINITRONE sensors =====================
        # XML sensor names (너가 추가한 Minitrone style)
        self.sid_quat = sid("body_quat")
        self.sid_gyro = sid("body_gyro")
        self.sid_pos  = sid("base_pos")
        self.sid_vel  = sid("base_linvel")

        # servo angles
        self.sid_servo_ang = [
            sid("servo1_angle"),
            sid("servo2_angle"),
            sid("servo3_angle"),
            sid("servo4_angle"),
        ]

        # for COM printing (use drone_base body)
        self.base_body_id = bid("drone_base")

        self.s_adr = self.model.sensor_adr
        self.s_dim = self.model.sensor_dim

        # -------- Input buffers --------
        self.ctrl_recv = np.zeros(8, dtype=float)
        self._delay_len = max(1, int(DELAY_TIME * PHYSICS_HZ))
        self._delay_buf = np.zeros((self._delay_len, 8), dtype=float)
        self._delay_idx = 0

        # -------- State memory --------
        self.prev_pub_t: Optional[float] = None
        self.prev_linvel_W: Optional[np.ndarray] = None
        self.prev_gyro_I: Optional[np.ndarray] = None

        # -------- ROS I/O --------
        self.sub_input = self.create_subscription(Input, "/minitrone/input", self.on_input, 10)
        self.pub_state = self.create_publisher(MinitroneState, "/minitrone/state", 10)

        self._lock = threading.Lock()
        self._stop = False

        self.sim_thread = threading.Thread(target=self.sim_loop, daemon=True)
        if self.enable_viewer:
            self.viewer_thread = threading.Thread(target=self.viewer_loop, daemon=True)
            self.viewer_thread.start()
        self.sim_thread.start()

        self.get_logger().info("[minitrone_plant] started (prop1~4, servo1~4)")

    def on_input(self, msg: Input):
        u = np.asarray(msg.u, dtype=float)
        if u.size < 8:
            return
        with self._lock:
            self.ctrl_recv = u[:8].copy()

    def _sensing(self, sid: int) -> np.ndarray:
        adr = self.s_adr[sid]
        dim = self.s_dim[sid]
        return np.array(self.data.sensordata[adr:adr+dim], dtype=float)

    def _noisy(self, x: np.ndarray, sigma: float) -> np.ndarray:
        return x + np.random.normal(0.0, sigma, size=x.shape)

    def _delay_step(self) -> np.ndarray:
        self._delay_buf[self._delay_idx] = self.ctrl_recv
        self._delay_idx = (self._delay_idx + 1) % self._delay_len
        return self._delay_buf[self._delay_idx]

    # -------- Simulation loop --------
    def sim_loop(self):
        next_step = time.perf_counter()
        next_pub = next_step

        while rclpy.ok() and not self._stop:
            now = time.perf_counter()

            with self._lock:
                u = self._delay_step()  # [omega1..4, servo1..4]

                # ---- props: ctrl = ZETA * omega^2 ----
                for i in range(4):
                    omega = float(u[i])
                    if omega < 0.0:
                        omega = 0.0
                    self.data.ctrl[self.aid_prop[i]] = ZETA * (omega * omega)

                # ---- servos: ctrl = desired angle (rad) ----
                for i in range(4):
                    self.data.ctrl[self.aid_servo[i]] = float(u[4 + i])

                # ---- step physics ----
                while now >= next_step:
                    mujoco.mj_step(self.model, self.data)
                    next_step += 1.0 / PHYSICS_HZ

                # ---- publish state ----
                while now >= next_pub:
                    quat_W = self._sensing(self.sid_quat)
                    gyro_I = self._noisy(self._sensing(self.sid_gyro), SIG_GYRO)
                    pos_W  = self._noisy(self._sensing(self.sid_pos),  SIG_POS)
                    vel_W  = self._noisy(self._sensing(self.sid_vel),  SIG_VEL)

                    rpy = quat_to_rpy(quat_W)

                    # COM debug print (optional)
                    if (now - self._last_com_print_t) >= COM_PRINT_PERIOD_S:
                        com_W = np.array(self.data.subtree_com[self.base_body_id], dtype=float)
                        R_WB = rpy_to_R_WB(rpy)
                        pc_B = R_WB.T @ (com_W - pos_W)
                        self.get_logger().info(
                            f"pc_B = [{pc_B[0]:.4f}, {pc_B[1]:.4f}, {pc_B[2]:.4f}]"
                        )
                        self._last_com_print_t = now

                    servo = self._noisy(
                        np.array([self._sensing(sid)[0] for sid in self.sid_servo_ang], dtype=float),
                        SIG_SERVO
                    )

                    t = now
                    if self.prev_pub_t is None:
                        acc_W = np.zeros(3, dtype=float)
                        a_rpy = np.zeros(3, dtype=float)
                    else:
                        dt = max(1e-6, t - self.prev_pub_t)
                        acc_W = (vel_W - self.prev_linvel_W) / dt
                        a_rpy = (gyro_I - self.prev_gyro_I) / dt

                    self.prev_pub_t = t
                    self.prev_linvel_W = vel_W.copy()
                    self.prev_gyro_I = gyro_I.copy()

                    msg = MinitroneState()
                    msg.pos   = pos_W.tolist()
                    msg.vel   = vel_W.tolist()
                    msg.acc   = acc_W.tolist()
                    msg.rpy   = rpy.tolist()
                    msg.w_rpy = gyro_I.tolist()
                    msg.a_rpy = a_rpy.tolist()
                    msg.servo = (servo * RAD2DEG).tolist()

                    self.pub_state.publish(msg)
                    next_pub += 1.0 / PHYSICS_HZ

            sleep_t = next_step - time.perf_counter()
            if sleep_t > 0:
                time.sleep(sleep_t)

    def viewer_loop(self):
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                while viewer.is_running() and rclpy.ok() and not self._stop:
                    with self._lock:
                        viewer.sync()
        except Exception as e:
            self.get_logger().warn(f"[viewer] ended: {e}")

    def close(self):
        self._stop = True


def main():
    rclpy.init()
    node = PlantRosNode()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
