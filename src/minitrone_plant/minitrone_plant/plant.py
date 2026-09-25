#!/usr/bin/env python3
import os, time, math, threading
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from minitrone_plant.high_fidelity import HighFidelityModel, PARAMETERS
from ament_index_python.packages import get_package_share_directory

import mujoco
import mujoco.viewer

from minitrone_interfaces.msg import CenterOfPressure, Input, MinitroneState, MobObserverInput, Wrench
from std_msgs.msg import Bool, Float64, Float64MultiArray, Int32

PHYSICS_HZ = 400.0
EXTERNAL_WRENCH_CMD_TIMEOUT = 0.2
RAD2DEG = 180.0 / math.pi

SIG_POS   = 1e-3
SIG_VEL   = 1e-3
SIG_GYRO  = 1e-3
SIG_SERVO = 1e-4

def quat_to_rpy(q_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = [float(v) for v in q_wxyz]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return np.array([roll, pitch, yaw], dtype=float)

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


def rotation_to_rpy(rotation: np.ndarray) -> np.ndarray:
    """Return XYZ roll/pitch/yaw for a WORLD-from-BODY rotation matrix."""
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
    yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    return np.array([roll, pitch, yaw], dtype=float)


class PlantRosNode(Node):
    def __init__(self):
        super().__init__("minitrone_plant")  # 이름 유지
        self.enable_viewer = bool(self.declare_parameter("enable_viewer", True).value)
        self.viewer_show_propellers = bool(
            self.declare_parameter(
                "viewer_show_propellers", True).value)
        self.viewer_contact_force_enabled = bool(
            self.declare_parameter(
                "viewer_show_contact_forces", True).value)
        self.viewer_contact_force_scale = max(
            0.0,
            float(self.declare_parameter(
                "viewer_contact_force_scale", 0.03).value),
        )
        self.viewer_contact_force_width = max(
            1e-4,
            float(self.declare_parameter(
                "viewer_contact_force_width", 0.008).value),
        )

        # -------- Load MuJoCo model --------
        pkg_share = get_package_share_directory("minitrone_plant")
        xml_path = os.path.join(pkg_share, "xml", "Scene.xml")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = 1.0 / PHYSICS_HZ
        random_seed = int(self.declare_parameter("random_seed", 1, ParameterDescriptor(read_only=True)).value)
        self.rng = np.random.default_rng(random_seed)
        # Startup-only parameters cannot silently change the ROS value without
        # rebuilding fixed uncertainty and queue state. Restart after editing.
        hf_config = {
            name: self.declare_parameter(
                name, spec[0], ParameterDescriptor(
                    read_only=True, description=f"{spec[2]} [{spec[1]}]; provisional")).value
            for name, spec in PARAMETERS.items()
        }
        self.high_fidelity = HighFidelityModel(hf_config, random_seed, 1.0 / PHYSICS_HZ)

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
        plate_size = np.asarray(
            self.model.geom_size[self.contact_plate_geom_id], dtype=float)
        plate_center = np.asarray(
            self.model.geom_pos[self.contact_plate_geom_id], dtype=float)
        plate_rotation_body_flat = np.empty(9, dtype=float)
        mujoco.mju_quat2Mat(
            plate_rotation_body_flat,
            self.model.geom_quat[self.contact_plate_geom_id],
        )
        self.rotation_body_contact = plate_rotation_body_flat.reshape(3, 3)
        # C is the center of the outward +X contact face, not the box geom
        # center. Read both pose and orientation from the loaded model.
        self.contact_center_body = (
            plate_center
            + self.rotation_body_contact
            @ np.array([plate_size[0], 0.0, 0.0], dtype=float)
        )
        self.cop_half_y = float(plate_size[1])
        self.cop_half_z = float(plate_size[2])
        self.cop_force_min = 0.5
        self.wall_body_id = bid("hand_palm")
        self.wall_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "hand_palm_col"
        )
        if self.wall_geom_id < 0:
            raise RuntimeError("Wall contact geom 'hand_palm_col' not found")
        # condim=3 is the normal Method-1 model. The flat plate already
        # produces pitch/yaw moments through its distributed contact forces;
        # condim=6 adds local rolling torques that can cancel those moments and
        # makes a wrench-derived CoP appear near the plate center. Case D
        # remains available as an explicit A/B diagnostic.
        self.contact_test_case = str(
            self.declare_parameter("contact_test_case", "B").value
        ).strip().upper()
        self.isolate_plate_wall_contact = bool(
            self.declare_parameter(
                "isolate_plate_wall_contact", True).value)
        self.contact_solref_timeconst = float(
            self.declare_parameter(
                "contact_solref_timeconst_sec", 0.0).value)
        self.contact_solref_dampratio = max(
            0.0, float(self.declare_parameter(
                "contact_solref_dampratio", 1.0).value))
        self.contact_margin = float(
            self.declare_parameter("contact_margin", -1.0).value)
        solver_iterations = int(
            self.declare_parameter("solver_iterations", 0).value)
        if solver_iterations > 0:
            self.model.opt.iterations = solver_iterations
        self._configure_contact_test_case()
        # Case A/B/C/D establishes the nominal pair first. The uncertainty layer
        # preserves those masks/condim and only changes friction and solref.
        self.high_fidelity.configure_physics(
            self.model, self.data, self.base_body_id,
            [self.wall_geom_id, self.contact_plate_geom_id])
        self.get_logger().info(self.high_fidelity.summary(random_seed))
        for geom_id in (self.wall_geom_id, self.contact_plate_geom_id):
            self.get_logger().info(
                f"[high_fidelity] actual contact geom={geom_id} "
                f"friction={self.model.geom_friction[geom_id].tolist()} "
                f"solref={self.model.geom_solref[geom_id].tolist()}")
        self.get_logger().info(
            f"[high_fidelity] plant base mass={self.model.body_mass[self.base_body_id]} kg; "
            f"principal J={self.model.body_inertia[self.base_body_id].tolist()} kg*m^2; "
            f"ipos={self.model.body_ipos[self.base_body_id].tolist()} m; controller nominal unchanged")
        # Keep the XML as an independent hard safety limit, even with HF off.
        # This also prevents overflow from malformed, excessively large omega.
        self.prop_thrust_limits = np.array([
            self.model.actuator_ctrlrange[i, 1] for i in self.aid_prop])
        if (not np.all(self.model.actuator_ctrllimited[self.aid_prop])
                or not np.isfinite(self.prop_thrust_limits).all()
                or np.any(self.prop_thrust_limits <= 0)):
            raise ValueError("BLDC actuators require finite positive XML ctrl limits")
        for index, actuator_id in enumerate(self.aid_prop):
            if self.model.actuator_forcelimited[actuator_id]:
                self.prop_thrust_limits[index] = min(
                    self.prop_thrust_limits[index],
                    self.model.actuator_forcerange[actuator_id, 1])
        if np.any(self.prop_thrust_limits <= 0):
            raise ValueError("BLDC force limits must be positive")
        self.get_logger().info(
            f"[high_fidelity] XML motor limits={self.prop_thrust_limits.tolist()} N; "
            f"configured plant limit={hf_config['motor_thrust_max']} N")
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

        # -------- State memory --------
        self.prev_true_vel = None
        self.prev_true_gyro = None
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
        self.pub_contact_wrench_gt = self.create_publisher(
            Wrench, "/contact_method1/contact_wrench_ground_truth_com", 10
        )
        self.pub_contact_wrench_center_gt = self.create_publisher(
            Wrench, "/contact_method1/contact_wrench_ground_truth_C", 10
        )
        self.pub_contact_count = self.create_publisher(
            Int32, "/contact_method1/contact_count", 10
        )
        self.pub_contact_points_body = self.create_publisher(
            Float64MultiArray, "/contact_method1/contact_points_body", 10
        )
        self.pub_plate_corner_gaps = self.create_publisher(
            Float64MultiArray, "/contact_method1/plate_corner_wall_gap", 10
        )
        self.pub_my_normal = self.create_publisher(
            Float64, "/contact_method1/debug/my_normal", 10
        )
        self.pub_my_tangential = self.create_publisher(
            Float64, "/contact_method1/debug/my_tangential", 10
        )
        self.pub_my_local = self.create_publisher(
            Float64, "/contact_method1/debug/my_local_condim", 10
        )
        self.pub_my_contact_total = self.create_publisher(
            Float64, "/contact_method1/debug/my_contact_total", 10
        )
        self.pub_contact_fn = self.create_publisher(
            Float64, "/contact_method1/debug/fn_ground_truth", 10
        )
        self.pub_contact_fz = self.create_publisher(
            Float64, "/contact_method1/debug/fz_ground_truth", 10
        )
        self.pub_contact_region = self.create_publisher(
            Int32, "/contact_method1/debug/contact_region", 10
        )
        self.pub_zero_contact = self.create_publisher(
            Bool, "/contact_method1/debug/zero_contact", 10
        )
        self.pub_contact_region_change_rate = self.create_publisher(
            Float64, "/contact_method1/debug/contact_region_change_rate", 10
        )
        self.pub_other_wall_contact_count = self.create_publisher(
            Int32, "/contact_method1/debug/other_wall_contact_count", 10
        )
        self.pub_relative_orientation_error = self.create_publisher(
            Float64MultiArray,
            "/contact_method1/debug/relative_orientation_error", 10
        )
        self.sub_palm_pose = self.create_subscription(
            Float64MultiArray, "/minitrone/palm_pose_cmd", self.on_palm_pose_cmd, 10
        )
        self.pub_palm_pose = self.create_publisher(Float64MultiArray, "/minitrone/palm_pose_state", 10)

        self._lock = threading.Lock()
        self._stop = False
        self._last_contact_region = 0
        self._contact_region_change_times = deque()

        self.sim_thread = threading.Thread(target=self.sim_loop, daemon=True)
        if self.enable_viewer:
            self.viewer_thread = threading.Thread(target=self.viewer_loop, daemon=True)
            self.viewer_thread.start()
        self.sim_thread.start()

        self.get_logger().info("[minitrone_plant] started (prop1~4, servo1~4)")

    def _configure_contact_test_case(self):
        """Select a repeatable wall-contact A/B model without editing XML."""
        case = self.contact_test_case
        if case not in {"A", "B", "C", "D"}:
            raise ValueError(
                "contact_test_case must be A, B, C, or D; "
                f"received {self.contact_test_case!r}"
            )

        if case == "D":
            self.get_logger().info(
                "contact test D: XML condim/friction unchanged"
            )
        else:
            condim = {"A": 1, "B": 3, "C": 6}[case]
            # Give the wall higher priority so its condim/friction wins when
            # MuJoCo combines it with the plate.
            self.model.geom_priority[self.wall_geom_id] = 1
            self.model.geom_condim[self.wall_geom_id] = condim
            self.model.geom_condim[self.contact_plate_geom_id] = condim

            if case == "A":
                friction = np.array([0.0, 0.0, 0.0], dtype=float)
            elif case == "B":
                friction = np.array([1.2, 0.0, 0.0], dtype=float)
            else:
                # condim=6 is retained while rolling/torsional friction is made
                # negligible. Sliding friction remains identical to Case B.
                friction = np.array([1.2, 1.0e-6, 1.0e-6], dtype=float)

            self.model.geom_friction[self.wall_geom_id, :] = friction
            self.model.geom_friction[self.contact_plate_geom_id, :] = friction
            self.get_logger().info(
                f"contact test {case}: condim={condim}, "
                f"friction={friction.tolist()}"
            )

            if self.isolate_plate_wall_contact:
                # Collision bit 2 is reserved for the Method-1 plate/wall pair.
                # Other drone geoms retain bit 1, so decorative/frame contacts
                # cannot duplicate the intended plate contact.
                for geom_id in (
                    self.wall_geom_id, self.contact_plate_geom_id):
                    self.model.geom_contype[geom_id] = 2
                    self.model.geom_conaffinity[geom_id] = 2
                # Compiled body collision masks are OR-reductions of geom masks.
                # Updating only geom bits leaves stale broad-phase masks and can
                # suppress the intended plate/palm pair entirely in A/B/C.
                for body_id in (self.base_body_id, self.wall_body_id):
                    geom_ids = np.flatnonzero(self.model.geom_bodyid == body_id)
                    self.model.body_contype[body_id] = np.bitwise_or.reduce(
                        self.model.geom_contype[geom_ids], initial=0)
                    self.model.body_conaffinity[body_id] = np.bitwise_or.reduce(
                        self.model.geom_conaffinity[geom_ids], initial=0)

        if self.contact_solref_timeconst > 0.0:
            minimum_timeconst = 2.0 * float(self.model.opt.timestep)
            if self.contact_solref_timeconst < minimum_timeconst:
                raise ValueError(
                    "contact_solref_timeconst_sec must be at least "
                    f"2*timestep={minimum_timeconst:.6f} s")
            # Positive solref format is (timeconst, dampratio). This is not the
            # negative direct (stiffness, damping) format used by the XML.
            for geom_id in (
                self.wall_geom_id, self.contact_plate_geom_id):
                self.model.geom_solref[geom_id, :] = np.array(
                    [self.contact_solref_timeconst,
                     self.contact_solref_dampratio], dtype=float)

        if self.contact_margin >= 0.0:
            for geom_id in (
                self.wall_geom_id, self.contact_plate_geom_id):
                self.model.geom_margin[geom_id] = self.contact_margin

        wall_solref = self.model.geom_solref[self.wall_geom_id].tolist()
        self.get_logger().info(
            "contact physics: "
            f"timestep={self.model.opt.timestep:.6f}s "
            f"iterations={self.model.opt.iterations} "
            f"wall_solref={wall_solref} "
            f"wall_solimp={self.model.geom_solimp[self.wall_geom_id].tolist()} "
            f"margin={self.model.geom_margin[self.wall_geom_id]:.6f} "
            f"isolated_pair={self.isolate_plate_wall_contact}"
        )

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
        return x + self.rng.normal(0.0, sigma, size=x.shape)

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

    def _actuation_wrench_body(self):
        # Preserve the observer API: motor wrench about the nominal BODY origin.
        # Actual CoM uncertainty belongs to unknown model mismatch; do not move
        # the controller's reference point to the sampled plant CoM implicitly.
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
        # xfrc_applied is WORLD force/torque at the actual body CoM. The two
        # BODY-axis external sources remain separate from actuator_force, so
        # the observer cannot accidentally cancel the unknown colored residual.
        residual = self.high_fidelity.external_residual()
        self.data.xfrc_applied[self.base_body_id, 0:3] = R_WB @ (self.external_force_body + residual[:3])
        self.data.xfrc_applied[self.base_body_id, 3:6] = R_WB @ (self.external_moment_body + residual[3:])

    def _measured_corner_loads(self) -> np.ndarray:
        """Distribute MuJoCo contact normal loads to four plate-corner load cells."""
        loads = np.zeros(4, dtype=float)
        contact_wrench = np.zeros(6, dtype=float)
        rotation_world_contact, contact_center_world = (
            self._contact_plate_frame_world()
        )

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

            contact_plate = (
                rotation_world_contact.T
                @ (np.asarray(contact.pos, dtype=float) - contact_center_world)
            )
            y_unit = float(np.clip(
                contact_plate[1] / self.cop_half_y, -1.0, 1.0))
            z_unit = float(np.clip(
                contact_plate[2] / self.cop_half_z, -1.0, 1.0))
            loads += normal_load * 0.25 * np.array([
                (1.0 + y_unit) * (1.0 + z_unit),
                (1.0 - y_unit) * (1.0 + z_unit),
                (1.0 - y_unit) * (1.0 - z_unit),
                (1.0 + y_unit) * (1.0 - z_unit),
            ])
        return loads

    def _contact_plate_frame_world(self):
        """Return WORLD-from-CONTACT rotation and the contact-face center."""
        rotation_world_contact = np.asarray(
            self.data.geom_xmat[self.contact_plate_geom_id],
            dtype=float,
        ).reshape(3, 3)
        plate_geom_center_world = np.asarray(
            self.data.geom_xpos[self.contact_plate_geom_id],
            dtype=float,
        )
        contact_center_world = (
            plate_geom_center_world
            + rotation_world_contact
            @ np.array([
                self.model.geom_size[self.contact_plate_geom_id, 0],
                0.0,
                0.0,
            ])
        )
        return rotation_world_contact, contact_center_world

    def _cop_from_corner_loads(self, loads: np.ndarray):
        """Return (normal force, y_C, z_C), or None below the CoP threshold."""
        normal_force = float(np.sum(loads))
        if normal_force < self.cop_force_min:
            return None
        corner_y = (
            np.array([1.0, -1.0, -1.0, 1.0], dtype=float)
            * self.cop_half_y
        )
        corner_z = (
            np.array([1.0, 1.0, -1.0, -1.0], dtype=float)
            * self.cop_half_z
        )
        cop_y = float(np.dot(loads, corner_y) / normal_force)
        cop_z = float(np.dot(loads, corner_z) / normal_force)
        return normal_force, cop_y, cop_z

    def _publish_cop_real(self):
        loads = self._measured_corner_loads()
        cop = self._cop_from_corner_loads(loads)
        msg = CenterOfPressure()
        msg.normal_force = float(np.sum(loads))
        msg.corner_forces = loads.tolist()
        msg.valid = cop is not None
        if msg.valid:
            _, msg.y, msg.z = cop
        self.pub_cop_real.publish(msg)

    def _plate_wall_contact_debug(self):
        """Publish validation-only MuJoCo contact geometry and wrench signals.

        The wrench is the wall-on-drone contact wrench, expressed in BODY and
        shifted to the drone CoM. It is never used as a controller input.

        ``mj_contactForce`` fills a spatial vector in CONTACT coordinates:
        [normal force, tangent-1 force, tangent-2 force,
         torsional torque, rolling-1 torque, rolling-2 torque].
        ``contact.frame`` stores contact axes in WORLD; its transpose below
        maps the contact-frame force/torque into WORLD.  The spatial wrench is
        treated as acting on geom2, and its sign is reversed when the drone
        plate is geom1.  These ground-truth values are diagnostic only.
        """
        base_pos_world = np.asarray(
            self.data.xpos[self.base_body_id], dtype=float)
        com_pos_world = np.asarray(self.data.xipos[self.base_body_id], dtype=float)
        rotation_world_body = np.asarray(
            self.data.xmat[self.base_body_id], dtype=float).reshape(3, 3)
        force_world = np.zeros(3, dtype=float)
        moment_com_world = np.zeros(3, dtype=float)
        moment_normal_world = np.zeros(3, dtype=float)
        moment_tangential_world = np.zeros(3, dtype=float)
        moment_local_world = np.zeros(3, dtype=float)
        normal_load_total = 0.0
        contact_points_body = []
        plate_wall_contact_count = 0
        other_wall_contact_count = 0
        contact_wrench = np.zeros(6, dtype=float)

        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            wall_in_contact = (
                contact.geom1 == self.wall_geom_id
                or contact.geom2 == self.wall_geom_id)
            plate_is_geom1 = (
                contact.geom1 == self.contact_plate_geom_id
                and contact.geom2 == self.wall_geom_id)
            plate_is_geom2 = (
                contact.geom2 == self.contact_plate_geom_id
                and contact.geom1 == self.wall_geom_id)
            is_plate_wall = plate_is_geom1 or plate_is_geom2
            if not is_plate_wall:
                if wall_in_contact:
                    other_wall_contact_count += 1
                continue

            mujoco.mj_contactForce(
                self.model, self.data, contact_index, contact_wrench)
            rotation_contact_world = np.asarray(
                contact.frame, dtype=float).reshape(3, 3).T
            torque_on_geom2_world = (
                rotation_contact_world @ contact_wrench[3:])
            sign_for_plate = (
                1.0 if contact.geom2 == self.contact_plate_geom_id else -1.0)
            force_normal_contact = np.array(
                [contact_wrench[0], 0.0, 0.0], dtype=float)
            force_tangential_contact = np.array(
                [0.0, contact_wrench[1], contact_wrench[2]], dtype=float)
            force_normal_plate_world = (
                sign_for_plate
                * rotation_contact_world
                @ force_normal_contact
            )
            force_tangential_plate_world = (
                sign_for_plate
                * rotation_contact_world
                @ force_tangential_contact
            )
            force_on_plate_world = (
                force_normal_plate_world + force_tangential_plate_world)
            torque_on_plate_world = sign_for_plate * torque_on_geom2_world
            contact_pos_world = np.asarray(contact.pos, dtype=float)
            lever_world = contact_pos_world - com_pos_world

            force_world += force_on_plate_world
            moment_normal_world += np.cross(
                lever_world, force_normal_plate_world)
            moment_tangential_world += np.cross(
                lever_world, force_tangential_plate_world)
            moment_local_world += torque_on_plate_world
            normal_load_total += max(0.0, float(contact_wrench[0]))
            contact_points_body.extend(
                (rotation_world_body.T
                 @ (contact_pos_world - base_pos_world)).tolist()
            )
            plate_wall_contact_count += 1

        moment_com_world = (
            moment_normal_world
            + moment_tangential_world
            + moment_local_world
        )
        force_body = rotation_world_body.T @ force_world
        moment_com_body = rotation_world_body.T @ moment_com_world
        moment_normal_body = rotation_world_body.T @ moment_normal_world
        moment_tangential_body = (
            rotation_world_body.T @ moment_tangential_world)
        moment_local_body = rotation_world_body.T @ moment_local_world
        wrench_msg = Wrench()
        wrench_msg.force = force_body.astype(np.float32).tolist()
        wrench_msg.moment = moment_com_body.astype(np.float32).tolist()
        self.pub_contact_wrench_gt.publish(wrench_msg)
        moment_center_body = (
            moment_com_body
            - np.cross(self.contact_center_body - self.model.body_ipos[self.base_body_id], force_body))
        center_wrench_msg = Wrench()
        center_wrench_msg.force = force_body.astype(np.float32).tolist()
        center_wrench_msg.moment = (
            moment_center_body.astype(np.float32).tolist())
        self.pub_contact_wrench_center_gt.publish(center_wrench_msg)

        count_msg = Int32()
        count_msg.data = plate_wall_contact_count
        self.pub_contact_count.publish(count_msg)
        count_msg.data = other_wall_contact_count
        self.pub_other_wall_contact_count.publish(count_msg)

        points_msg = Float64MultiArray()
        points_msg.data = contact_points_body
        self.pub_contact_points_body.publish(points_msg)

        if plate_wall_contact_count == 0:
            contact_region = 0  # no plate/wall contact
        else:
            point_matrix = np.asarray(
                contact_points_body, dtype=float).reshape(-1, 3)
            centroid_z = float(np.mean(point_matrix[:, 2]))
            if centroid_z < -0.02:
                contact_region = 1  # lower edge
            elif centroid_z > 0.02:
                contact_region = 3  # upper edge
            else:
                contact_region = 2  # centered or both edges

        sim_time = float(self.data.time)
        if contact_region != self._last_contact_region:
            self._contact_region_change_times.append(sim_time)
            self._last_contact_region = contact_region
        while (
            self._contact_region_change_times
            and self._contact_region_change_times[0] < sim_time - 1.0
        ):
            self._contact_region_change_times.popleft()

        region_msg = Int32()
        region_msg.data = contact_region
        self.pub_contact_region.publish(region_msg)
        zero_msg = Bool()
        zero_msg.data = plate_wall_contact_count == 0
        self.pub_zero_contact.publish(zero_msg)
        rate_msg = Float64()
        rate_msg.data = float(len(self._contact_region_change_times))
        self.pub_contact_region_change_rate.publish(rate_msg)

        for publisher, value in (
            (self.pub_my_normal, moment_normal_body[1]),
            (self.pub_my_tangential, moment_tangential_body[1]),
            (self.pub_my_local, moment_local_body[1]),
            (self.pub_my_contact_total, moment_com_body[1]),
            (self.pub_contact_fn, normal_load_total),
            (self.pub_contact_fz, force_body[2]),
        ):
            scalar_msg = Float64()
            scalar_msg.data = float(value)
            publisher.publish(scalar_msg)

        # Signed distance from each plate corner to the near wall face.
        # Positive: separated, zero: touching, negative: penetration.
        wall_center_world = np.asarray(
            self.data.xpos[self.wall_body_id], dtype=float)
        rotation_world_wall = np.asarray(
            self.data.xmat[self.wall_body_id], dtype=float).reshape(3, 3)
        wall_outward_normal = -rotation_world_wall[:, 0]
        wall_near_face = wall_center_world - 0.02 * rotation_world_wall[:, 0]
        corner_gaps = []
        for y_coord, z_coord in (
            (self.cop_half_y, self.cop_half_z),
            (-self.cop_half_y, self.cop_half_z),
            (-self.cop_half_y, -self.cop_half_z),
            (self.cop_half_y, -self.cop_half_z),
        ):
            corner_body = self.contact_center_body + np.array(
                [0.0, y_coord, z_coord], dtype=float)
            corner_world = (
                base_pos_world + rotation_world_body @ corner_body)
            corner_gaps.append(float(
                np.dot(corner_world - wall_near_face, wall_outward_normal)))
        gaps_msg = Float64MultiArray()
        gaps_msg.data = corner_gaps
        self.pub_plate_corner_gaps.publish(gaps_msg)

        # Validation-only surface orientation error.  The wall and plate are
        # parallel when WORLD-from-WALL^T * WORLD-from-BODY is identity.
        relative_rotation = rotation_world_wall.T @ rotation_world_body
        relative_msg = Float64MultiArray()
        relative_msg.data = rotation_to_rpy(relative_rotation).tolist()
        self.pub_relative_orientation_error.publish(relative_msg)

    # -------- Simulation loop --------
    def sim_loop(self):
        try:
            self._sim_loop_impl()
        except RCLError:
            # SIGINT invalidates the ROS context before the background thread
            # necessarily finishes its current publish batch.
            if rclpy.ok() and not self._stop:
                raise

    def _physics_step(self):
        """Advance every model exactly once per 2.5 ms of simulation time."""
        thrust, angles = self.high_fidelity.actuate(
            float(self.data.time), self.ctrl_recv, self.prop_thrust_limits)
        self.data.ctrl[self.aid_prop] = thrust
        self.data.ctrl[self.aid_servo] = angles
        self._apply_palm_pose_cmd()
        self._apply_external_wrench()
        mujoco.mj_step(self.model, self.data)
        # mj_step integrates qpos after computing sensors; refresh to pair the
        # published state, contact geometry and actuator wrench at the same pose.
        mujoco.mj_forward(self.model, self.data)
        invalid_warnings = (mujoco.mjtWarning.mjWARN_BADQPOS,
                            mujoco.mjtWarning.mjWARN_BADQVEL,
                            mujoco.mjtWarning.mjWARN_BADQACC)
        # MuJoCo can auto-reset after a numerical failure, leaving finite arrays.
        # Inspect warning counters as well so that a reset cannot look successful.
        if (not np.isfinite(self.data.qpos).all()
                or not np.isfinite(self.data.qvel).all()
                or not np.isfinite(self.data.qacc).all()
                or any(self.data.warning[int(w)].number for w in invalid_warnings)):
            raise RuntimeError("invalid MuJoCo state or numerical-reset warning")

    def _measure_state(self):
        quat_W = self._sensing(self.sid_quat)
        true_gyro = self._sensing(self.sid_gyro)
        true_pos = self._sensing(self.sid_pos)
        true_vel = self._sensing(self.sid_vel)
        if self.high_fidelity.enabled('inertial_uncertainty'):
            # XML framepos/framelinvel on BODY refer to the inertial CoM. Once
            # ipos moves, keep the sensor at the original body-frame location
            # rather than making the simulated tracker follow randomized CoM.
            true_pos = np.array(self.data.xpos[self.base_body_id], copy=True)
            velocity = np.zeros(6)
            mujoco.mj_objectVelocity(
                self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY,
                self.base_body_id, velocity, 0)
            true_vel = velocity[3:]
        true_servo = np.array([self._sensing(sid)[0] for sid in self.sid_servo_ang])
        t = float(self.data.time)
        dt = 1.0 / PHYSICS_HZ
        if self.high_fidelity.enabled('sensor_model'):
            # Differentiate truth at the fixed physics period BEFORE sampling.
            # Differentiating noisy/held 100 Hz velocity at 400 Hz creates fake
            # acceleration spikes. acc is body IMU specific force per the msg.
            acc_W = np.zeros(3) if self.prev_true_vel is None else (true_vel - self.prev_true_vel) / dt
            alpha = np.zeros(3) if self.prev_true_gyro is None else (true_gyro - self.prev_true_gyro) / dt
            rotation = np.asarray(self.data.xmat[self.base_body_id]).reshape(3, 3)
            truth = dict(position=true_pos, velocity=true_vel,
                         attitude=quat_to_rpy(quat_W), gyro=true_gyro,
                         servo=true_servo,
                         acceleration=rotation.T @ (acc_W - self.model.opt.gravity),
                         angular_acceleration=alpha)
            sensed = self.high_fidelity.measure(t, truth)
            pos_W, vel_W = sensed['position'], sensed['velocity']
            rpy, gyro_I = sensed['attitude'], sensed['gyro']
            servo = sensed['servo']
            acc_W, a_rpy = sensed['acceleration'], sensed['angular_acceleration']
        else:
            # Retain legacy white-noise channels and legacy world acceleration
            # convention on OFF; the HF sensor path fixes the IMU convention.
            gyro_I = self._noisy(true_gyro, SIG_GYRO)
            pos_W = self._noisy(true_pos, SIG_POS)
            vel_W = self._noisy(true_vel, SIG_VEL)
            rpy = quat_to_rpy(quat_W)
            servo = self._noisy(true_servo, SIG_SERVO)
            acc_W = np.zeros(3) if self.prev_pub_t is None else (vel_W - self.prev_linvel_W) / dt
            a_rpy = np.zeros(3) if self.prev_pub_t is None else (gyro_I - self.prev_gyro_I) / dt
        self.prev_pub_t = t
        self.prev_linvel_W = vel_W.copy()
        self.prev_gyro_I = gyro_I.copy()
        self.prev_true_vel = true_vel.copy()
        self.prev_true_gyro = true_gyro.copy()
        return pos_W, vel_W, acc_W, rpy, gyro_I, a_rpy, servo

    def _publish_state(self):
        pos_W, vel_W, acc_W, rpy, gyro_I, a_rpy, servo = self._measure_state()
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
        self._plate_wall_contact_debug()
        self._publish_palm_pose()

    def _sim_loop_impl(self):
        next_step = time.perf_counter()
        while rclpy.ok() and not self._stop:
            now = time.perf_counter()
            # Wall time only paces execution. Queues, noise and actuator states
            # advance on simulation ticks, including every catch-up step.
            while now >= next_step and rclpy.ok() and not self._stop:
                with self._lock:
                    self._physics_step()
                    self._publish_state()
                next_step += 1.0 / PHYSICS_HZ
            sleep_t = next_step - time.perf_counter()
            if sleep_t > 0:
                time.sleep(sleep_t)  # Existing real-time pacing, not sensor latency.

    def _viewer_key_callback(self, keycode):
        """Toggle plate contact-force arrows when F is pressed."""
        if keycode != ord("F"):
            return
        self.viewer_contact_force_enabled = (
            not self.viewer_contact_force_enabled)
        state = "ON" if self.viewer_contact_force_enabled else "OFF"
        self.get_logger().info(
            f"[viewer] plate contact-force arrows {state} (F=toggle)")

    def _plate_contact_resultant_world(self):
        """Return wall-on-plate force applied at the published real CoP."""
        force_world = np.zeros(3, dtype=float)
        contact_wrench = np.zeros(6, dtype=float)

        for contact_index in range(self.data.ncon):
            contact = self.data.contact[contact_index]
            plate_is_geom1 = (
                contact.geom1 == self.contact_plate_geom_id
                and contact.geom2 == self.wall_geom_id)
            plate_is_geom2 = (
                contact.geom2 == self.contact_plate_geom_id
                and contact.geom1 == self.wall_geom_id)
            if not (plate_is_geom1 or plate_is_geom2):
                continue

            mujoco.mj_contactForce(
                self.model, self.data, contact_index, contact_wrench)
            rotation_contact_world = np.asarray(
                contact.frame, dtype=float).reshape(3, 3).T
            # MuJoCo's extracted contact wrench follows the geom2 sign in this
            # contact frame. Negate it when the plate is geom1 so the arrow
            # always represents the wall force acting on the drone plate.
            plate_sign = 1.0 if plate_is_geom2 else -1.0
            force_on_plate_world = (
                plate_sign
                * rotation_contact_world
                @ contact_wrench[:3]
            )
            force_world += force_on_plate_world

        # Use exactly the same corner-load calculation as /minitrone/cop_real.
        # The point is projected onto the nominal contact face x_C=0, so
        # MuJoCo penetration depth cannot move the arrow origin along x_C.
        cop = self._cop_from_corner_loads(self._measured_corner_loads())
        if cop is None:
            return None
        _, cop_y, cop_z = cop
        rotation_world_contact, contact_center_world = (
            self._contact_plate_frame_world()
        )
        cop_world = (
            contact_center_world
            + rotation_world_contact
            @ np.array([0.0, cop_y, cop_z], dtype=float)
        )
        return cop_world, force_world

    def _update_viewer_contact_force_arrow(self, viewer):
        """Populate the viewer user scene with the plate resultant-force arrow."""
        viewer.user_scn.ngeom = 0
        if not self.viewer_contact_force_enabled:
            return

        resultant = self._plate_contact_resultant_world()
        if resultant is None or viewer.user_scn.maxgeom < 1:
            return
        origin_world, force_world = resultant
        force_norm = float(np.linalg.norm(force_world))
        if force_norm <= 1e-6:
            return

        arrow_length = min(
            0.50,
            self.viewer_contact_force_scale * force_norm,
        )
        arrow_end_world = (
            origin_world + arrow_length * force_world / force_norm)
        arrow_geom = viewer.user_scn.geoms[0]
        mujoco.mjv_initGeom(
            arrow_geom,
            mujoco.mjtGeom.mjGEOM_ARROW,
            np.zeros(3, dtype=float),
            np.zeros(3, dtype=float),
            np.eye(3, dtype=float).reshape(-1),
            np.array([1.0, 0.15, 0.05, 0.95], dtype=np.float32),
        )
        mujoco.mjv_connector(
            arrow_geom,
            mujoco.mjtGeom.mjGEOM_ARROW,
            self.viewer_contact_force_width,
            origin_world,
            arrow_end_world,
        )
        arrow_geom.category = mujoco.mjtCatBit.mjCAT_DECOR
        viewer.user_scn.ngeom = 1

    def _set_viewer_status_text(self, viewer):
        """Show the force-arrow shortcut and current state in the viewer."""
        state = "ON" if self.viewer_contact_force_enabled else "OFF"
        viewer.set_texts((
            mujoco.mjtFontScale.mjFONTSCALE_100,
            mujoco.mjtGridPos.mjGRID_TOPLEFT,
            "F: CoP contact-force arrow",
            state,
        ))

    def _configure_viewer_options(self, viewer):
        """Apply this simulation's default viewer visibility groups."""
        # Propeller discs and blades use visual geom groups 1~4. MuJoCo leaves
        # some of these groups disabled by default.
        last_prop_group = min(5, len(viewer.opt.geomgroup))
        viewer.opt.geomgroup[1:last_prop_group] = (
            self.viewer_show_propellers)

    def viewer_loop(self):
        try:
            # launch_passive runs mj_forward and loads the shared mjData.
            # Exclude physics updates until viewer initialization completes.
            with self._lock:
                viewer = mujoco.viewer.launch_passive(
                    self.model,
                    self.data,
                    key_callback=self._viewer_key_callback,
                )
            with viewer:
                with viewer.lock():
                    self._configure_viewer_options(viewer)
                self._set_viewer_status_text(viewer)
                previous_force_arrow_state = (
                    self.viewer_contact_force_enabled)
                while viewer.is_running() and rclpy.ok() and not self._stop:
                    force_arrow_state_changed = False
                    with self._lock:
                        with viewer.lock():
                            self._update_viewer_contact_force_arrow(viewer)
                        force_arrow_state_changed = (
                            previous_force_arrow_state
                            != self.viewer_contact_force_enabled)
                        # sync copies shared mjData and applies GUI inputs.
                        # It takes the viewer lock internally, but also needs
                        # our physics lock to exclude concurrent mj_step.
                        viewer.sync()
                    if force_arrow_state_changed:
                        self._set_viewer_status_text(viewer)
                        previous_force_arrow_state = (
                            self.viewer_contact_force_enabled)
        except Exception as e:
            self.get_logger().warn(f"[viewer] ended: {e}")

    def close(self):
        self._stop = True
        if (
            hasattr(self, "sim_thread")
            and self.sim_thread.is_alive()
            and threading.current_thread() is not self.sim_thread
        ):
            self.sim_thread.join(timeout=1.0)


def main():
    rclpy.init()
    node = PlantRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
