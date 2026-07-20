#!/usr/bin/env python3
import os, time, math, threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import mujoco
import mujoco.viewer

from minitrone_interfaces.msg import CenterOfPressure, Input, MinitroneState, MobObserverInput, Wrench
from std_msgs.msg import Float64MultiArray

PHYSICS_HZ = 400.0
ZETA = 0.02          # thrust = ZETA * omega^2  (minitrone allocator/plant convention)
DELAY_TIME = 0.01
EXTERNAL_WRENCH_CMD_TIMEOUT = 0.2
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


def rpy_to_quat_wxyz(rpy: np.ndarray) -> np.ndarray:
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = math.cos(0.5 * r), math.sin(0.5 * r)
    cp, sp = math.cos(0.5 * p), math.sin(0.5 * p)
    cy, sy = math.cos(0.5 * y), math.sin(0.5 * y)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
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

        def siteid(name: str) -> int:
            idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name.encode())
            if idx < 0:
                raise RuntimeError(f"Site '{name}' not found in XML")
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
        self.contact_plate_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "contact_plate_px"
        )
        if self.contact_plate_geom_id < 0:
            raise RuntimeError("Contact geom 'contact_plate_px' not found in XML")
        self.cop_half_y = 0.200
        self.cop_half_z = 0.190
        self.cop_force_min = 0.5
        self.prop_site_id = [
            siteid("prop1_site"),
            siteid("prop2_site"),
            siteid("prop3_site"),
            siteid("prop4_site"),
        ]

        self.palm_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "hand_palm")
        self.palm_mocap_id = -1
        self.palm_pose_cmd = None
        self.palm_home_pos = np.array([1.4, 0.0, 1.0], dtype=float)
        self.aid_palm = []
        if self.palm_body_id >= 0:
            self.palm_mocap_id = int(self.model.body_mocapid[self.palm_body_id])
            if self.palm_mocap_id >= 0:
                self.data.mocap_pos[self.palm_mocap_id] = self.model.body_pos[self.palm_body_id]
                self.data.mocap_quat[self.palm_mocap_id] = self.model.body_quat[self.palm_body_id]
            else:
                self.palm_home_pos = np.array([1.4, 0.0, 1.0], dtype=float)
                self.palm_pose_cmd = np.concatenate((self.palm_home_pos, np.zeros(3, dtype=float)))
                self.aid_palm = [
                    aid("hand_palm_slide_x_pos"),
                    aid("hand_palm_slide_y_pos"),
                    aid("hand_palm_slide_z_pos"),
                    aid("hand_palm_roll_pos"),
                    aid("hand_palm_pitch_pos"),
                    aid("hand_palm_yaw_pos"),
                ]
        else:
            self.get_logger().warn("Body 'hand_palm' not found; palm teleop disabled")

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
        self.external_force_body = np.zeros(3, dtype=float)
        self.external_moment_body = np.zeros(3, dtype=float)
        self.last_external_wrench_cmd_wall_t: Optional[float] = None

        # -------- ROS I/O --------
        self.sub_input = self.create_subscription(Input, "/minitrone/input", self.on_input, 10)
        self.sub_external_wrench = self.create_subscription(
            Wrench, "/minitrone/external_wrench_cmd", self.on_external_wrench, 10
        )
        self.pub_state = self.create_publisher(MinitroneState, "/minitrone/state", 10)
        self.pub_mob_observer_input = self.create_publisher(
            MobObserverInput, "/minitrone/mob_observer_input", 10
        )
        self.pub_actuation_wrench_body = self.create_publisher(
            Wrench, "/minitrone/actuation_wrench_body", 10
        )
        self.pub_cop_real = self.create_publisher(
            CenterOfPressure, "/minitrone/cop_real", 10
        )
        self.sub_palm_pose = self.create_subscription(
            Float64MultiArray, "/minitrone/palm_pose_cmd", self.on_palm_pose_cmd, 10
        )
        self.pub_palm_pose = self.create_publisher(Float64MultiArray, "/minitrone/palm_pose_state", 10)

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

    def on_external_wrench(self, msg: Wrench):
        with self._lock:
            self.external_force_body[:] = np.asarray(msg.force, dtype=float)
            self.external_moment_body[:] = np.asarray(msg.moment, dtype=float)
            self.last_external_wrench_cmd_wall_t = time.perf_counter()

    def on_palm_pose_cmd(self, msg: Float64MultiArray):
        if self.palm_body_id < 0 or len(msg.data) < 6:
            return
        if self.palm_mocap_id < 0:
            with self._lock:
                self.palm_pose_cmd = np.array(msg.data[:6], dtype=float)
            return
        pos = np.array(msg.data[:3], dtype=float)
        quat = rpy_to_quat_wxyz(np.array(msg.data[3:6], dtype=float))
        with self._lock:
            self.data.mocap_pos[self.palm_mocap_id] = pos
            self.data.mocap_quat[self.palm_mocap_id] = quat

    def _sensing(self, sid: int) -> np.ndarray:
        adr = self.s_adr[sid]
        dim = self.s_dim[sid]
        return np.array(self.data.sensordata[adr:adr+dim], dtype=float)

    def _noisy(self, x: np.ndarray, sigma: float) -> np.ndarray:
        return x + np.random.normal(0.0, sigma, size=x.shape)

    def _publish_palm_pose(self):
        if self.palm_body_id < 0:
            return
        if self.palm_mocap_id >= 0:
            pos = np.array(self.data.mocap_pos[self.palm_mocap_id], dtype=float)
            quat = np.array(self.data.mocap_quat[self.palm_mocap_id], dtype=float)
        else:
            pos = np.array(self.data.xpos[self.palm_body_id], dtype=float)
            quat = np.array(self.data.xquat[self.palm_body_id], dtype=float)
        rpy = quat_to_rpy(quat)
        msg = Float64MultiArray()
        msg.data = np.concatenate((pos, rpy)).tolist()
        self.pub_palm_pose.publish(msg)

    def _apply_palm_pose_cmd(self):
        if self.palm_mocap_id >= 0 or not self.aid_palm or self.palm_pose_cmd is None:
            return
        rel_pos = np.asarray(self.palm_pose_cmd[:3], dtype=float) - self.palm_home_pos
        rpy_cmd = np.asarray(self.palm_pose_cmd[3:6], dtype=float)
        palm_ctrl = np.array([
            rel_pos[0],
            rel_pos[1],
            rel_pos[2],
            rpy_cmd[0],
            rpy_cmd[1],
            rpy_cmd[2],
        ], dtype=float)
        for actuator_id, ctrl in zip(self.aid_palm, palm_ctrl):
            self.data.ctrl[actuator_id] = float(ctrl)

    def _delay_step(self) -> np.ndarray:
        self._delay_buf[self._delay_idx] = self.ctrl_recv
        self._delay_idx = (self._delay_idx + 1) % self._delay_len
        return self._delay_buf[self._delay_idx]

    def _actuation_wrench_body(self):
        base_pos_W = np.asarray(self.data.xpos[self.base_body_id], dtype=float)
        R_WB = np.asarray(self.data.xmat[self.base_body_id], dtype=float).reshape(3, 3)
        force_W = np.zeros(3, dtype=float)
        moment_W = np.zeros(3, dtype=float)

        for actuator_id, site_id in zip(self.aid_prop, self.prop_site_id):
            actuator_force = float(self.data.actuator_force[actuator_id])
            gear = np.asarray(self.model.actuator_gear[actuator_id], dtype=float)
            R_WS = np.asarray(self.data.site_xmat[site_id], dtype=float).reshape(3, 3)
            site_pos_W = np.asarray(self.data.site_xpos[site_id], dtype=float)

            prop_force_W = R_WS @ (gear[:3] * actuator_force)
            prop_moment_W = (
                np.cross(site_pos_W - base_pos_W, prop_force_W)
                + R_WS @ (gear[3:6] * actuator_force)
            )
            force_W += prop_force_W
            moment_W += prop_moment_W

        return R_WB.T @ force_W, R_WB.T @ moment_W

    def _apply_external_wrench(self):
        if (
            self.last_external_wrench_cmd_wall_t is not None
            and time.perf_counter() - self.last_external_wrench_cmd_wall_t
            > EXTERNAL_WRENCH_CMD_TIMEOUT
        ):
            self.external_force_body[:] = 0.0
            self.external_moment_body[:] = 0.0

        R_WB = np.asarray(self.data.xmat[self.base_body_id], dtype=float).reshape(3, 3)
        self.data.xfrc_applied[self.base_body_id, :] = 0.0
        self.data.xfrc_applied[self.base_body_id, 0:3] = R_WB @ self.external_force_body
        self.data.xfrc_applied[self.base_body_id, 3:6] = R_WB @ self.external_moment_body

    def _measured_corner_loads(self) -> np.ndarray:
        """Distribute MuJoCo contact normal loads to four plate-corner load cells."""
        loads = np.zeros(4, dtype=float)
        base_pos = np.asarray(self.data.xpos[self.base_body_id], dtype=float)
        r_bw = np.asarray(self.data.xmat[self.base_body_id], dtype=float).reshape(3, 3).T
        contact_wrench = np.zeros(6, dtype=float)

        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            if (
                contact.geom1 != self.contact_plate_geom_id
                and contact.geom2 != self.contact_plate_geom_id
            ):
                continue
            mujoco.mj_contactForce(self.model, self.data, contact_index, contact_wrench)
            normal_load = max(0.0, float(contact_wrench[0]))
            if normal_load <= 0.0:
                continue

            contact_body = r_bw @ (np.asarray(contact.pos, dtype=float) - base_pos)
            y_unit = float(np.clip(contact_body[1] / self.cop_half_y, -1.0, 1.0))
            z_unit = float(np.clip(contact_body[2] / self.cop_half_z, -1.0, 1.0))
            loads += normal_load * 0.25 * np.array([
                (1.0 + y_unit) * (1.0 + z_unit),
                (1.0 - y_unit) * (1.0 + z_unit),
                (1.0 - y_unit) * (1.0 - z_unit),
                (1.0 + y_unit) * (1.0 - z_unit),
            ])
        return loads

    def _publish_cop_real(self):
        loads = self._measured_corner_loads()
        normal_force = float(np.sum(loads))
        msg = CenterOfPressure()
        msg.normal_force = normal_force
        msg.corner_forces = loads.tolist()
        msg.valid = normal_force >= self.cop_force_min
        if msg.valid:
            corner_y = np.array([1.0, -1.0, -1.0, 1.0]) * self.cop_half_y
            corner_z = np.array([1.0, 1.0, -1.0, -1.0]) * self.cop_half_z
            msg.y = float(np.dot(loads, corner_y) / normal_force)
            msg.z = float(np.dot(loads, corner_z) / normal_force)
        self.pub_cop_real.publish(msg)

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

                self._apply_palm_pose_cmd()
                self._apply_external_wrench()

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

                    actuation_force_B, actuation_moment_B = self._actuation_wrench_body()
                    mob_msg = MobObserverInput()
                    mob_msg.step = int(round(float(self.data.time) * PHYSICS_HZ))
                    mob_msg.sim_time = float(self.data.time)
                    mob_msg.pos = pos_W.tolist()
                    mob_msg.vel = vel_W.tolist()
                    mob_msg.rpy = rpy.tolist()
                    mob_msg.w_rpy = gyro_I.tolist()
                    mob_msg.actuation_force = actuation_force_B.astype(np.float32).tolist()
                    mob_msg.actuation_moment = actuation_moment_B.astype(np.float32).tolist()
                    self.pub_mob_observer_input.publish(mob_msg)

                    actuation_wrench_msg = Wrench()
                    actuation_wrench_msg.force = actuation_force_B.astype(np.float32).tolist()
                    actuation_wrench_msg.moment = actuation_moment_B.astype(np.float32).tolist()
                    self.pub_actuation_wrench_body.publish(actuation_wrench_msg)
                    self._publish_cop_real()
                    self._publish_palm_pose()
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
